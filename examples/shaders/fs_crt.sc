$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

uniform vec4 u_time;
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;
uniform vec4 u_dimensions;
uniform vec4 u_crt_params;  // x: intensity (0-1 ensemble), y: unused, z: unused, w: unused

// CRT effect intensity controls how many effects are applied:
// 0.0 = no effect (passthrough)
// 0.0-0.25: subtle scanlines only
// 0.25-0.5: scanlines + light chromatic aberration + subtle jitter
// 0.5-0.75: scanlines + chromatic aberration + curvature + jitter
// 0.75-1.0: full CRT (all above + vignette + noise + stronger jitter)

// Hash function for pseudo-random values
float hash(float n) {
  return fract(sin(n) * 43758.5453123);
}

// Smooth noise for jitter
float smoothNoise(float x) {
  float i = floor(x);
  float f = fract(x);
  return mix(hash(i), hash(i + 1.0), f * f * (3.0 - 2.0 * f));
}

vec2 curveUV(vec2 uv, float curvature) {
  // Convert to centered coordinates
  vec2 centered = uv * 2.0 - 1.0;
  
  // Apply barrel distortion
  float r2 = dot(centered, centered);
  centered *= 1.0 + curvature * r2;
  
  // Convert back
  return centered * 0.5 + 0.5;
}

// Horizontal jitter - displaces each scanline horizontally based on time and position
float horizontalJitter(float y, float timeVal, float jitterStrength) {
  // Multiple frequency components for organic feel
  float slowWave = smoothNoise(y * 3.0 + timeVal * 0.5) - 0.5;
  float fastWave = smoothNoise(y * 20.0 + timeVal * 8.0) - 0.5;
  
  // Occasional larger glitches (rare events)
  float glitchTrigger = smoothNoise(y * 0.5 + timeVal * 2.0);
  float glitch = glitchTrigger > 0.92 ? (hash(y + timeVal) - 0.5) * 4.0 : 0.0;
  
  // Combine: slow drift + fast jitter + occasional glitch
  return (slowWave * 0.3 + fastWave * 0.7 + glitch) * jitterStrength;
}

vec3 chromaticAberration(vec2 uv, float amount) {
  // Get screen center offset
  vec2 centered = uv - 0.5;
  float dist = length(centered);
  
  // Offset increases towards edges
  vec2 offset = centered * dist * amount;
  
  // Sample RGB with offsets
  float r = texture2D(s_texture, clamp(uv + offset, u_texture_clamp.xy, u_texture_clamp.zw)).r;
  float g = texture2D(s_texture, clamp(uv, u_texture_clamp.xy, u_texture_clamp.zw)).g;
  float b = texture2D(s_texture, clamp(uv - offset, u_texture_clamp.xy, u_texture_clamp.zw)).b;
  
  return vec3(r, g, b);
}

float scanline(vec2 uv, float count, float intensity) {
  // Create horizontal scanlines
  float scanVal = sin(uv.y * count * kPi * 2.0);
  scanVal = scanVal * 0.5 + 0.5;  // 0-1 range
  
  // Add subtle flicker based on time for analog feel
  float flicker = 1.0 + 0.01 * sin(u_time.x * 8.0);
  
  // Mix between full brightness and scanline darkness
  return mix(1.0, 0.7 + 0.3 * scanVal, intensity) * flicker;
}

float vignette(vec2 uv, float intensity) {
  vec2 centered = uv - 0.5;
  float dist = length(centered) * 1.41421356;  // Normalize to corner = 1
  float vig = 1.0 - dist * dist * intensity;
  return clamp(vig, 0.0, 1.0);
}

float grainNoise(vec2 uv) {
  return fract(sin(dot(uv, vec2(12.9898, 78.233)) + u_time.x * 0.1) * 43758.5453);
}

void main() {
  float intensity = u_crt_params.x;
  
  // Early out for zero intensity
  if (intensity <= 0.001) {
    gl_FragColor = texture2D(s_texture, v_texture_uv);
    gl_FragColor.rgb *= u_color_mult.rgb;
    return;
  }
  
  vec2 uv = v_texture_uv;
  
  // Calculate effect strengths based on ensemble intensity
  // Each effect fades in at different ranges for smooth progression
  float scanline_strength = smoothstep(0.0, 0.3, intensity);
  float chroma_strength = smoothstep(0.15, 0.5, intensity) * 0.015;
  float jitter_strength = smoothstep(0.2, 0.6, intensity) * 0.008;  // Subtle jitter starts early
  float curve_strength = smoothstep(0.4, 0.75, intensity) * 0.08;
  float vignette_strength = smoothstep(0.5, 0.9, intensity) * 0.4;
  float noise_strength = smoothstep(0.7, 1.0, intensity) * 0.02;
  
  // At very high intensity, increase jitter for more glitchy look
  jitter_strength += smoothstep(0.8, 1.0, intensity) * 0.015;
  
  // Apply horizontal jitter (per-scanline displacement)
  if (jitter_strength > 0.0001) {
    float jitterOffset = horizontalJitter(uv.y * u_dimensions.y, u_time.x, jitter_strength);
    uv.x += jitterOffset;
  }
  
  // Apply curvature distortion
  if (curve_strength > 0.001) {
    uv = curveUV(uv, curve_strength);
    
    // Check if we're outside the valid texture area (black bars)
    if (uv.x < u_texture_clamp.x || uv.x > u_texture_clamp.z ||
        uv.y < u_texture_clamp.y || uv.y > u_texture_clamp.w) {
      gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
      return;
    }
  }
  
  // Sample with chromatic aberration
  vec3 color;
  if (chroma_strength > 0.0001) {
    color = chromaticAberration(uv, chroma_strength);
  } else {
    color = texture2D(s_texture, clamp(uv, u_texture_clamp.xy, u_texture_clamp.zw)).rgb;
  }
  
  // Apply scanlines - use screen pixel position for consistent line spacing
  float scanline_count = u_dimensions.y * 0.5;  // One scanline per 2 pixels
  float scan = scanline(uv, scanline_count, scanline_strength * 0.5);
  color *= scan;
  
  // Apply vignette
  color *= vignette(uv, vignette_strength);
  
  // Apply subtle noise/grain
  if (noise_strength > 0.001) {
    float n = grainNoise(uv * u_dimensions.xy) * 2.0 - 1.0;
    color += n * noise_strength;
  }
  
  // Apply color multiplier
  color *= u_color_mult.rgb;
  
  gl_FragColor = vec4(color, 1.0);
}

vec4 v_color0         : COLOR0    = vec4(1.0, 0.0, 0.0, 1.0);
vec4 v_color1         : COLOR1    = vec4(0.0, 1.0, 0.0, 1.0);
vec4 v_color2         : COLOR2    = vec4(0.0, 0.0, 1.0, 1.0);
vec4 v_color3         : COLOR3    = vec4(1.0, 0.0, 1.0, 1.0);
vec2 v_coordinates    : TEXCOORD0 = vec2(0.0, 0.0);
vec2 v_texture_uv     : TEXCOORD0 = vec2(0.0, 0.0);
vec2 v_dimensions     : TEXCOORD1 = vec2(0.0, 0.0);
vec4 v_shader_values  : TEXCOORD2 = vec4(0.0, 0.0, 0.0, 0.0);
vec4 v_shader_values1 : TEXCOORD3 = vec4(0.0, 0.0, 0.0, 0.0);

vec2 a_position      : POSITION;
vec4 a_color0        : COLOR0;
vec4 a_color1        : COLOR1;
vec4 a_color2        : COLOR2;
vec4 a_color3        : COLOR3;
vec4 a_texcoord0     : TEXCOORD0;
vec4 a_texcoord1     : TEXCOORD1;
vec4 a_texcoord2     : TEXCOORD2;
vec4 a_texcoord3     : TEXCOORD3;

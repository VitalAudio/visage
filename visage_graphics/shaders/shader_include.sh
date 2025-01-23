#if BGFX_SHADER_LANGUAGE_HLSL || BGFX_SHADER_LANGUAGE_PSSL || BGFX_SHADER_LANGUAGE_SPIRV || BGFX_SHADER_LANGUAGE_METAL
#	define fract(_x) frac(_x)

#	if BGFX_SHADER_LANGUAGE_HLSL > 400
#		define REGISTER(_type, _reg) register( _type[_reg] )
#	else
#		define REGISTER(_type, _reg) register(_type ## _reg)
#	endif // BGFX_SHADER_LANGUAGE_HLSL

#	if BGFX_SHADER_LANGUAGE_HLSL > 300 || BGFX_SHADER_LANGUAGE_PSSL || BGFX_SHADER_LANGUAGE_SPIRV || BGFX_SHADER_LANGUAGE_METAL
struct BgfxSampler2D {
	SamplerState m_sampler;
	Texture2D m_texture;
};

vec4 internalTexture2D(BgfxSampler2D _sampler, vec2 _coord) {
	return _sampler.m_texture.Sample(_sampler.m_sampler, _coord);
}

#		define SAMPLER2D(_name, _reg) \
			uniform SamplerState _name ## Sampler : REGISTER(s, _reg); \
			uniform Texture2D _name ## Texture : REGISTER(t, _reg); \
			static BgfxSampler2D _name = { _name ## Sampler, _name ## Texture }
#		define sampler2D BgfxSampler2D
#		define texture2D(_sampler, _coord) internalTexture2D(_sampler, _coord)
#	else
#		define SAMPLER2D(_name, _reg) uniform sampler2D _name : REGISTER(s, _reg)
#		define texture2D(_sampler, _coord) tex2D(_sampler, _coord)
#	endif // BGFX_SHADER_LANGUAGE_HLSL > 300

float mix(float _a, float _b, float _t) { return lerp(_a, _b, _t); }
vec2  mix(vec2  _a, vec2  _b, vec2  _t) { return lerp(_a, _b, _t); }
vec3  mix(vec3  _a, vec3  _b, vec3  _t) { return lerp(_a, _b, _t); }
vec4  mix(vec4  _a, vec4  _b, vec4  _t) { return lerp(_a, _b, _t); }

float mod(float _a, float _b) { return _a - _b * floor(_a / _b); }
vec2  mod(vec2  _a, vec2  _b) { return _a - _b * floor(_a / _b); }
vec3  mod(vec3  _a, vec3  _b) { return _a - _b * floor(_a / _b); }
vec4  mod(vec4  _a, vec4  _b) { return _a - _b * floor(_a / _b); }

#else
#	define CONST(_x) const _x
#	define atan2(_x, _y) atan(_x, _y)
#	define mul(_a, _b) ( (_a) * (_b) )
#	define saturate(_x) clamp(_x, 0.0, 1.0)
#	define SAMPLER2D(_name, _reg)       uniform sampler2D _name

#	if BGFX_SHADER_LANGUAGE_GLSL >= 130
#		define texture2D(_sampler, _coord)      texture(_sampler, _coord)
#	endif // BGFX_SHADER_LANGUAGE_GLSL >= 130
#endif // BGFX_SHADER_LANGUAGE_*

vec2 vec2_splat(float _x) { return vec2(_x, _x); }
vec3 vec3_splat(float _x) { return vec3(_x, _x, _x); }
vec4 vec4_splat(float _x) { return vec4(_x, _x, _x, _x); }

#define kPi 3.141592653589793238462643383279
#define kArcRounded 1.5
#define kArcFlat 0.5

#include <shader_utils.sh>

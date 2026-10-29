#define CONST(_x) const _x
#define atan2(_x, _y) atan(_x, _y)
#define mul(_a, _b) ( (_a) * (_b) )
#define saturate(_x) clamp(_x, 0.0, 1.0)
#define texture2D(_sampler, _coord) texture(_sampler, _coord)
#define texture2DLod(_sampler, _coord, _level) textureLod(_sampler, _coord, _level)

vec2 vec2_splat(float _x) { return vec2(_x, _x); }
vec3 vec3_splat(float _x) { return vec3(_x, _x, _x); }
vec4 vec4_splat(float _x) { return vec4(_x, _x, _x, _x); }

#define kPi 3.141592653589793238462643383279
#define kArcRounded 1.5
#define kArcFlat 0.5

#include <shader_utils.sh>

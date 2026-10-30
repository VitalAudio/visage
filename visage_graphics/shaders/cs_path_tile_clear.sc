#include <bgfx_compute.sh>

BUFFER_RW(b_tiles, uint, 0);
BUFFER_RW(b_counter, uint, 1);
BUFFER_RW(b_covers, uint, 2);

uniform vec4 u_tile_params;

NUM_THREADS(64, 1, 1)
void main() {
  uint index = gl_GlobalInvocationID.x + gl_GlobalInvocationID.y * uint(u_tile_params.y);
  uint count = uint(u_tile_params.x);
  if (index == 0u)
    b_counter[0] = 0u;
  if (index < count / 4u)
    b_tiles[index] = 0u;
  if (index < count)
    b_covers[index] = 0u;
}

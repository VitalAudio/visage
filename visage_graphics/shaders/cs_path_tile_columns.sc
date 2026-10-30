#include <bgfx_compute.sh>

BUFFER_RO(b_paths, vec4, 0);
BUFFER_RW(b_tiles, uint, 1);
BUFFER_RW(b_counter, uint, 2);
BUFFER_RO(b_covers, uint, 3);
IMAGE2D_WO(i_segments, rgba32f, 4);
IMAGE2D_WO(i_tiles, rgba32f, 5);

uniform vec4 u_tile_params;
uniform vec4 u_tile_layout;

#define kTileSize 16
#define kCoverScale 65536.0

ivec2 texel(uint index) {
  uint width = uint(u_tile_layout.x);
  return ivec2(int(index % width), int(index / width));
}

NUM_THREADS(64, 1, 1)
void main() {
  uint index = gl_GlobalInvocationID.x + gl_GlobalInvocationID.y * uint(u_tile_params.y);
  if (index >= uint(u_tile_params.x))
    return;

  int low = 0;
  int high = int(u_tile_params.z + 0.5) - 1;
  while (low < high) {
    int middle = (low + high + 1) / 2;
    if (b_paths[uint(middle) * 3u + 1u].w <= float(index))
      low = middle;
    else
      high = middle - 1;
  }

  vec4 cell = b_paths[uint(low) * 3u];
  vec4 grid = b_paths[uint(low) * 3u + 1u];
  vec4 rule = b_paths[uint(low) * 3u + 2u];
  int tile_base = int(grid.x + 0.5);
  int columns = int(grid.y + 0.5);
  int rows = int(grid.z + 0.5);
  int column = int(index) - int(grid.w + 0.5);

  int backdrop = 0;
  uint total = 0u;
  for (int row = rows - 1; row >= 0; --row) {
    uint tile = uint(tile_base + row * columns + column);
    backdrop += int(b_tiles[tile * 4u + 3u]);
    b_tiles[tile * 4u + 3u] = uint(backdrop);
    total += b_tiles[tile * 4u];
  }

  uint offset;
  atomicFetchAndAdd(b_counter[0], total, offset);

  vec2 cell_end = cell.xy + cell.zw;
  for (int row = 0; row < rows; ++row) {
    uint tile = uint(tile_base + row * columns + column);
    uint count = b_tiles[tile * 4u];
    b_tiles[tile * 4u + 1u] = offset;

    int cover = 0;
    bool covered = false;
    for (uint pixel = 0u; pixel < 16u; ++pixel) {
      cover += int(b_covers[tile * 16u + pixel]);
      covered = covered || cover != 0;
    }

    float cover_texel = -1.0;
    if (covered) {
      uint cover_base = uint(u_tile_layout.y) + tile * 4u;
      cover_texel = float(cover_base);
      cover = 0;
      for (uint group = 0u; group < 4u; ++group) {
        vec4 values;
        cover += int(b_covers[tile * 16u + group * 4u]);
        values.x = float(cover) / kCoverScale;
        cover += int(b_covers[tile * 16u + group * 4u + 1u]);
        values.y = float(cover) / kCoverScale;
        cover += int(b_covers[tile * 16u + group * 4u + 2u]);
        values.z = float(cover) / kCoverScale;
        cover += int(b_covers[tile * 16u + group * 4u + 3u]);
        values.w = float(cover) / kCoverScale;
        imageStore(i_segments, texel(cover_base + group), values);
      }
    }

    vec2 origin = cell.xy + vec2(float(column), float(row)) * float(kTileSize);
    vec2 end = min(origin + vec2_splat(float(kTileSize)), cell_end);
    imageStore(i_tiles, texel(tile * 3u), vec4(float(offset), float(count), float(int(b_tiles[tile * 4u + 3u])), cover_texel));
    imageStore(i_tiles, texel(tile * 3u + 1u), vec4(origin, end));
    imageStore(i_tiles, texel(tile * 3u + 2u), rule);
    offset += count;
  }
}

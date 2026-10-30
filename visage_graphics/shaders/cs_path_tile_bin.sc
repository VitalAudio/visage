#include <bgfx_compute.sh>

BUFFER_RO(b_edges, vec4, 0);
BUFFER_RO(b_edge_paths, uint, 1);
BUFFER_RO(b_paths, vec4, 2);
BUFFER_RW(b_tiles, uint, 3);
IMAGE2D_WO(i_segments, rgba32f, 4);
BUFFER_RW(b_covers, uint, 5);

uniform vec4 u_tile_params;
uniform vec4 u_tile_layout;

#define kTileSize 16.0
#define kNoCrossing 3.4e38
#define kCoverScale 65536.0

int tileCell(float value, int count) {
  float cell = floor(value * (1.0 / kTileSize));
  if (cell < 0.0)
    return -1;
  if (cell >= float(count))
    return count;
  return int(cell);
}

void addSegment(int tile, vec4 segment) {
  uint index = uint(tile) * 4u;
  if (u_tile_params.z < 0.5)
    atomicAdd(b_tiles[index], 1u);
  else {
    uint slot;
    atomicFetchAndAdd(b_tiles[index + 2u], 1u, slot);
    uint position = b_tiles[index + 1u] + slot;
    uint width = uint(u_tile_layout.x);
    if (position < uint(u_tile_params.w))
      imageStore(i_segments, ivec2(int(position % width), int(position / width)), segment);
  }
}

void columnCrossing(int column, int row, int direction, ivec3 grid) {
  if (u_tile_params.z < 0.5 && column >= 0 && column < grid.y && row > 0) {
    int tile = grid.x + (min(row, grid.z) - 1) * grid.y + column;
    atomicAdd(b_tiles[uint(tile) * 4u + 3u], uint(direction));
  }
}

void rowCrossing(int column, int row, float x, int direction, ivec3 grid) {
  if (u_tile_params.z > 0.5 || column < 0 || column >= grid.y || row < 0 || row >= grid.z)
    return;

  float local_x = clamp(x - float(column) * kTileSize, 0.0, kTileSize);
  if (local_x == kTileSize)
    return;

  uint index = uint(grid.x + row * grid.y + column) * 16u;
  int pixel = int(local_x);
  float fraction = local_x - float(pixel);
  atomicAdd(b_covers[index + uint(pixel)], uint(int(round(float(direction) * (1.0 - fraction) * kCoverScale))));
  if (pixel < 15)
    atomicAdd(b_covers[index + uint(pixel + 1)], uint(int(round(float(direction) * fraction * kCoverScale))));
}

void addPiece(vec2 start, vec2 end, int column, int row, ivec3 grid) {
  if ((start.x != end.x || start.y != end.y) && column >= 0 && column < grid.y && row >= 0 && row < grid.z) {
    vec2 origin = vec2(float(column), float(row)) * kTileSize;
    addSegment(grid.x + row * grid.y + column, vec4(start - origin, end - origin));
  }
}

NUM_THREADS(64, 1, 1)
void main() {
  uint index = gl_GlobalInvocationID.x + gl_GlobalInvocationID.y * uint(u_tile_params.y);
  if (index >= uint(u_tile_params.x))
    return;

  vec4 edge = b_edges[index];
  vec4 path_grid = b_paths[b_edge_paths[index] * 3u + 1u];
  ivec3 grid = ivec3(int(path_grid.x + 0.5), int(path_grid.y + 0.5), int(path_grid.z + 0.5));
  vec2 from = edge.xy;
  vec2 to = edge.zw;

  int column = tileCell(from.x, grid.y);
  int row = tileCell(from.y, grid.z);
  int end_column = tileCell(to.x, grid.y);
  int end_row = tileCell(to.y, grid.z);
  int x_step = end_column > column ? 1 : -1;
  int y_step = end_row > row ? 1 : -1;
  int x_line = column + (x_step > 0 ? 1 : 0);
  int y_line = row + (y_step > 0 ? 1 : 0);
  int x_remaining = abs(end_column - column);
  int y_remaining = abs(end_row - row);
  float dx = to.x - from.x;
  float dy = to.y - from.y;
  vec2 previous = from;
  while (x_remaining > 0 || y_remaining > 0) {
    float x_t = x_remaining > 0 ? (float(x_line) * kTileSize - from.x) / dx : kNoCrossing;
    float y_t = y_remaining > 0 ? (float(y_line) * kTileSize - from.y) / dy : kNoCrossing;
    if (x_t <= y_t) {
      vec2 crossing = vec2(float(x_line) * kTileSize, from.y + x_t * dy);
      addPiece(previous, crossing, column, row, grid);
      previous = crossing;
      columnCrossing(x_line, row, x_step, grid);
      column += x_step;
      x_line += x_step;
      x_remaining -= 1;
    }
    else {
      vec2 crossing = vec2(from.x + y_t * dx, float(y_line) * kTileSize);
      addPiece(previous, crossing, column, row, grid);
      previous = crossing;
      rowCrossing(column, min(row, row + y_step), crossing.x, y_step, grid);
      row += y_step;
      y_line += y_step;
      y_remaining -= 1;
    }
  }

  addPiece(previous, to, column, row, grid);
}

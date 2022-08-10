#ifdef VERTEX_SHADER
in vec2 vert_position;
in vec2 vert_tex_coord;
out vec2 frag_tex_coord;
layout(location = 0) uniform mat3x2 modelview_matrix;
void main() {
    gl_Position = vec4(mat2(modelview_matrix) * vert_position + modelview_matrix[2], 0.0, 1.0);
    frag_tex_coord = vert_tex_coord;
}
#endif

#ifdef FRAGMENT_SHADER
in vec2 frag_tex_coord;
out vec4 color;
layout(binding = 0) uniform sampler2D color_texture;
layout(location = 1) uniform vec2 resolution;
layout(location = 2) uniform float time;
void main() {
    color = texture(color_texture, frag_tex_coord);
}
#endif

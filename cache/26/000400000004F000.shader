// shader: 8DD9, BF8D50647A6521FD

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
struct Vertex {
    vec4 attributes[3];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(0.0, 0.0, 0.0, 0.0);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);
    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[3](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0]);
    prim_buffer[1].attributes = vec4[3](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1]);
    prim_buffer[2].attributes = vec4[3](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 5AAA17FAADBF3883, BF8D50647A6521FD
// shader: 8B31, 2468EC2880A72F75

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp1);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp1);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp1);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp1);
vs_out_attr1 = vs_in_reg1.xyyy;
return false;
}
bool sub_0() {
sub_1();
vs_out_attr2 = vs_in_reg2;
return true;
}
// reference: 2889471BCB6725AA, 2468EC2880A72F75
// shader: 8B30, C63DC6A6AE67F97B
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rrr) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.b) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5DEB14BF50878544, C63DC6A6AE67F97B
// program: 2468EC2880A72F75, BF8D50647A6521FD, C63DC6A6AE67F97B
// shader: 8B31, 53D7B04016D617D0

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp1);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp1);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp1);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp1);
vs_out_attr1 = vs_in_reg1.xyyy;
return false;
}
bool sub_0() {
sub_1();
vs_out_attr2 = vs_in_reg2.xyzz;
return true;
}
// reference: 2889471BFC6E7470, 53D7B04016D617D0
// shader: 8B30, B1606F52FF782F6B
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A0370CB1FD9D7721, B1606F52FF782F6B
// program: 53D7B04016D617D0, BF8D50647A6521FD, B1606F52FF782F6B
// shader: 8DD9, AAC8D6E0C4449CAB

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
struct Vertex {
    vec4 attributes[2];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(0.0, 0.0, 0.0, 0.0);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(0.0, 0.0, 0.0, 0.0);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);
    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[2](vs_out_attr0[0], vs_out_attr1[0]);
    prim_buffer[1].attributes = vec4[2](vs_out_attr0[1], vs_out_attr1[1]);
    prim_buffer[2].attributes = vec4[2](vs_out_attr0[2], vs_out_attr1[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: EB4D5A09C6660182, AAC8D6E0C4449CAB
// shader: 8B31, 914E6F276D627A75

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0 = vs_in_reg0;
reg_tmp0.zw = (uniforms.f[92]).zw;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp0);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp0);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp0);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp0);
vs_out_attr1 = vs_in_reg0.zwww;
return true;
}
// reference: 87CDD55C5F7FEF96, 914E6F276D627A75
// shader: 8B30, A9FC8F2549C65973
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FA2CBCEC702F0E81, A9FC8F2549C65973
// program: 914E6F276D627A75, AAC8D6E0C4449CAB, A9FC8F2549C65973
// shader: 8DD9, 410E4C5F77308E32

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
layout(location=5) in vec4 vs_out_attr5[];
struct Vertex {
    vec4 attributes[6];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(0.0, 0.0, 0.0, 0.0);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z);
    texcoord2 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[6](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0]);
    prim_buffer[1].attributes = vec4[6](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1]);
    prim_buffer[2].attributes = vec4[6](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 3DD7010810511CAC, 410E4C5F77308E32
// shader: 8B31, A6D17325FF79E0F8

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = reg_tmp0.xyyy;
vs_out_attr4 = reg_tmp0.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg4.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg4.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg4.xyz);
reg_tmp13.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp13.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp13.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
sub_2();
return false;
}
bool sub_2() {
uint jmp_to = 80u;
while (true) {
switch (jmp_to) {
case 80u:
vs_out_attr3 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 116u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 154u; break;
}
case 116u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_3();
} else {
sub_8();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 154u:
vs_out_attr2 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
if (bool_regs.y) {
sub_4();
} else {
sub_5();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_4() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_5() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_6();
} else {
sub_7();
}
return false;
}
bool sub_6() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_7() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_8() {
if (bool_regs.y) {
sub_9();
} else {
sub_10();
}
return false;
}
bool sub_9() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_0() {
vs_out_attr5 = vs_in_reg2.xyyy;
reg_tmp0 = vs_in_reg1;
sub_1();
return true;
}
// reference: D3ED44EDC4480525, A6D17325FF79E0F8
// shader: 8B30, D069A48998DA2A17
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = texcolor0;
light_vector = normalize(light_src[1].position + view);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.ggg) * (texcolor1.rrr), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.ggg) * (last_tex_env_out.rrr) + (last_tex_env_out.rrr), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ECAF1EE997FD8349, D069A48998DA2A17
// program: A6D17325FF79E0F8, 410E4C5F77308E32, D069A48998DA2A17
// reference: D3ED44ED117F169D, A6D17325FF79E0F8
// shader: 8B30, 886ED2A12BE673E0
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = texcolor0;
light_vector = normalize(light_src[1].position + view);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.ggg) * (texcolor1.rrr), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.ggg) * (last_tex_env_out.rrr) + (last_tex_env_out.rrr), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ECAF1EE951765B21, 886ED2A12BE673E0
// program: A6D17325FF79E0F8, 410E4C5F77308E32, 886ED2A12BE673E0
// shader: 8B31, 0C3BA222B5416BED

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = reg_tmp0.xyyy;
vs_out_attr4 = reg_tmp0.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg4.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg4.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg4.xyz);
reg_tmp13.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp13.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp13.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
sub_2();
return false;
}
bool sub_2() {
uint jmp_to = 80u;
while (true) {
switch (jmp_to) {
case 80u:
vs_out_attr3 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 116u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 154u; break;
}
case 116u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_3();
} else {
sub_8();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 154u:
vs_out_attr2 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
if (bool_regs.y) {
sub_4();
} else {
sub_5();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_4() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_5() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_6();
} else {
sub_7();
}
return false;
}
bool sub_6() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_7() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_8() {
if (bool_regs.y) {
sub_9();
} else {
sub_10();
}
return false;
}
bool sub_9() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_0() {
reg_tmp0 = vs_in_reg1;
vs_out_attr5 = vs_in_reg5.xyyy;
sub_1();
return true;
}
// reference: D3ED44EDB0E17473, 0C3BA222B5416BED
// shader: 8B30, 986D4BBB3020D140
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(vec3(dot((vec3(1) - texcolor1.rgb) - vec3(0.5), (const_color[0].rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (vec3(1) - last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CEE0946B2C46F1E1, 986D4BBB3020D140
// program: 0C3BA222B5416BED, 410E4C5F77308E32, 986D4BBB3020D140
// shader: 8DD9, 3CDADE164F37EFAA

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
struct Vertex {
    vec4 attributes[4];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(0.0, 0.0, 0.0, 0.0);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z);
    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[4](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0]);
    prim_buffer[1].attributes = vec4[4](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1]);
    prim_buffer[2].attributes = vec4[4](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 63ACA0372EFCEE74, 3CDADE164F37EFAA
// shader: 8B31, AB6AD9F21067B359

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = vs_in_reg1.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp2.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp2.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp2.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp2.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp1.x = dot_s(uniforms.f[8], reg_tmp2);
reg_tmp1.y = dot_s(uniforms.f[9], reg_tmp2);
reg_tmp1.z = dot_s(uniforms.f[10], reg_tmp2);
reg_tmp1.w = dot_s(uniforms.f[11], reg_tmp2);
reg_tmp13.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp13.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp13.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp13.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp13.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp13.xyz);
vs_out_attr3 = -reg_tmp1;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp1);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp1);
bool_regs = equal(-uniforms.f[93].yy, reg_tmp14.zz);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp1);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp1);
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
vs_out_attr2.w = (uniforms.f[93].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (!bool_regs.x) {
sub_2();
} else {
sub_3();
}
return false;
}
bool sub_2() {
vs_out_attr2.z = rcp_s(reg_tmp4.x);
vs_out_attr2.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_3() {
vs_out_attr2 = uniforms.f[93].yxxx;
return false;
}
bool sub_0() {
sub_1();
return true;
}
// reference: EEA9014788F09FF9, AB6AD9F21067B359
// shader: 8B30, 519CECC94364C41C
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rrr) + (const_color[0].rrr) - vec3(0.5), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor1.rrr) * (const_color[1].rrr), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a < alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CCEBF14FD939FB60, 519CECC94364C41C
// program: AB6AD9F21067B359, 3CDADE164F37EFAA, 519CECC94364C41C
// shader: 8DD9, 0E1DAAC943BB278F

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
layout(location=5) in vec4 vs_out_attr5[];
struct Vertex {
    vec4 attributes[6];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = vtx.attributes[5].z;
    view = vec3(vtx.attributes[4].x, vtx.attributes[4].y, vtx.attributes[4].z);
    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[6](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0]);
    prim_buffer[1].attributes = vec4[6](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1]);
    prim_buffer[2].attributes = vec4[6](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 946EFCFAC1D0879F, 0E1DAAC943BB278F
// shader: 8B31, 2F6AB1438B14E98E

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = vs_in_reg1.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp13.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp13.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp13.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp13.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp13.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp13.xyz);
vs_out_attr4 = -reg_tmp15;
reg_tmp0 = vec4(dot_3(-reg_tmp15.xyz, reg_tmp14.xyz));
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp0 = reg_tmp0 + reg_tmp0;
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp0 = fma_s(reg_tmp0, reg_tmp14, reg_tmp15);
vs_out_attr3.w = (uniforms.f[93].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (!bool_regs.x) {
sub_2();
} else {
sub_3();
}
vs_out_attr5.x = (reg_tmp0.xxxx).x;
vs_out_attr5.y = (reg_tmp0.yyyy).y;
vs_out_attr5.z = (reg_tmp0.zzzz).z;
vs_out_attr5.w = (uniforms.f[93].xxxx).w;
return false;
}
bool sub_2() {
vs_out_attr3.z = rcp_s(reg_tmp4.x);
vs_out_attr3.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_3() {
vs_out_attr3.xyz = (uniforms.f[93].yxxx).xyz;
return false;
}
bool sub_0() {
reg_tmp0 = vs_in_reg2;
reg_tmp0.w = (vs_in_reg4.xxxx).w;
vs_out_attr2 = reg_tmp0;
sub_1();
return true;
}
// reference: 9E82D571AAD99CB0, 2F6AB1438B14E98E
// shader: 8B30, 6DC1C6BDCC9811A8
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa) + (texture(tex_cube, vec3(texcoord0, texcoord0_w)).rgb) * (vec3(1) - (const_color[1].aaa)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1) - (vec3(1) - rounded_primary_color.aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (primary_fragment_color.aaa), vec3(1)) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8370AEBF94A94800, 6DC1C6BDCC9811A8
// program: 2F6AB1438B14E98E, 0E1DAAC943BB278F, 6DC1C6BDCC9811A8
// shader: 8DD9, 66C01D9968A40A5A

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
layout(location=5) in vec4 vs_out_attr5[];
layout(location=6) in vec4 vs_out_attr6[];
struct Vertex {
    vec4 attributes[7];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[6].x, vtx.attributes[6].y);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = vtx.attributes[6].z;
    view = vec3(vtx.attributes[4].x, vtx.attributes[4].y, vtx.attributes[4].z);
    texcoord2 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[7](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0], vs_out_attr6[0]);
    prim_buffer[1].attributes = vec4[7](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1], vs_out_attr6[1]);
    prim_buffer[2].attributes = vec4[7](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2], vs_out_attr6[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 2FF8FE6E72C4ECA1, 66C01D9968A40A5A
// shader: 8B31, 1A640611D16050C2

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;
layout(location=3) in vec4 vs_in_reg3;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = vs_in_reg1.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp13.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp13.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp13.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp13.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp13.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp13.xyz);
vs_out_attr4 = -reg_tmp15;
reg_tmp0 = vec4(dot_3(-reg_tmp15.xyz, reg_tmp14.xyz));
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp0 = reg_tmp0 + reg_tmp0;
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp0 = fma_s(reg_tmp0, reg_tmp14, reg_tmp15);
vs_out_attr3.w = (uniforms.f[93].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (!bool_regs.x) {
sub_2();
} else {
sub_3();
}
vs_out_attr6.x = (reg_tmp0.xxxx).x;
vs_out_attr6.y = (reg_tmp0.yyyy).y;
vs_out_attr6.z = (reg_tmp0.zzzz).z;
vs_out_attr6.w = (uniforms.f[93].xxxx).w;
return false;
}
bool sub_2() {
vs_out_attr3.z = rcp_s(reg_tmp4.x);
vs_out_attr3.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_3() {
vs_out_attr3.xyz = (uniforms.f[93].yxxx).xyz;
return false;
}
bool sub_0() {
vs_out_attr5 = vs_in_reg1.xyyy;
vs_out_attr2 = vs_in_reg2;
sub_1();
return true;
}
// reference: 897A2053C741342F, 1A640611D16050C2
// shader: 8B30, B495600476852DA8
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (vec3(1) - texcolor2.rrr) + (texture(tex_cube, vec3(texcoord0, texcoord0_w)).rgb) * (vec3(1) - (vec3(1) - texcolor2.rrr)), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor2.rrr), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9A636E0994D87557, B495600476852DA8
// program: 1A640611D16050C2, 66C01D9968A40A5A, B495600476852DA8
// shader: 8DD9, CF9491352B19EDBB

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
layout(location=5) in vec4 vs_out_attr5[];
struct Vertex {
    vec4 attributes[6];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[4].x, vtx.attributes[4].y, vtx.attributes[4].z);
    texcoord2 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[6](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0]);
    prim_buffer[1].attributes = vec4[6](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1]);
    prim_buffer[2].attributes = vec4[6](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 98DDD608C1957939, CF9491352B19EDBB
// shader: 8B31, 2A6CA064843AACD5

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = vs_in_reg1.xyyy;
vs_out_attr5 = vs_in_reg1.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp15.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp15.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp15.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp15.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp14.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp14.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp14.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp13.x = dot_3(uniforms.f[4].xyz, vs_in_reg4.xyz);
reg_tmp13.y = dot_3(uniforms.f[5].xyz, vs_in_reg4.xyz);
reg_tmp13.z = dot_3(uniforms.f[6].xyz, vs_in_reg4.xyz);
sub_2();
return false;
}
bool sub_2() {
uint jmp_to = 80u;
while (true) {
switch (jmp_to) {
case 80u:
vs_out_attr4 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 116u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 154u; break;
}
case 116u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_3();
} else {
sub_8();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 154u:
vs_out_attr3 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
if (bool_regs.y) {
sub_4();
} else {
sub_5();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_4() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_5() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_6();
} else {
sub_7();
}
return false;
}
bool sub_6() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_7() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_8() {
if (bool_regs.y) {
sub_9();
} else {
sub_10();
}
return false;
}
bool sub_9() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_0() {
vs_out_attr2 = vs_in_reg2;
sub_1();
return true;
}
// reference: 263EB2148CAEE230, 2A6CA064843AACD5
// shader: 8B30, F77AAEE911D965A7
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[4].position);
spot_dir = light_src[4].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[4].diffuse * dot_product) + light_src[4].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[4].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[4].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F42CAA708598C4A6, F77AAEE911D965A7
// program: 2A6CA064843AACD5, CF9491352B19EDBB, F77AAEE911D965A7
// reference: 2889471B2CEF37AF, 53D7B04016D617D0
// shader: 8B30, 5F84AE56F4383170
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rrr) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.b);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 335F822DFD9D7721, 5F84AE56F4383170
// program: 53D7B04016D617D0, BF8D50647A6521FD, 5F84AE56F4383170
// shader: 8DD9, 5840B6EDF6A7B4C9

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
struct Vertex {
    vec4 attributes[5];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[4].x, vtx.attributes[4].y, vtx.attributes[4].z);
    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[5](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0]);
    prim_buffer[1].attributes = vec4[5](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1]);
    prim_buffer[2].attributes = vec4[5](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: AC86E947C6112B83, 5840B6EDF6A7B4C9
// shader: 8B31, 707D664331545344

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;
layout(location=3) in vec4 vs_in_reg3;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = vs_in_reg1.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp2.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp2.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp2.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp2.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp1.x = dot_s(uniforms.f[8], reg_tmp2);
reg_tmp1.y = dot_s(uniforms.f[9], reg_tmp2);
reg_tmp1.z = dot_s(uniforms.f[10], reg_tmp2);
reg_tmp1.w = dot_s(uniforms.f[11], reg_tmp2);
reg_tmp13.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp13.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp13.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp13.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp13.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp13.xyz);
vs_out_attr4 = -reg_tmp1;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp1);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp1);
bool_regs = equal(-uniforms.f[93].yy, reg_tmp14.zz);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp1);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp1);
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
vs_out_attr3.w = (uniforms.f[93].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (!bool_regs.x) {
sub_2();
} else {
sub_3();
}
return false;
}
bool sub_2() {
vs_out_attr3.z = rcp_s(reg_tmp4.x);
vs_out_attr3.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_3() {
vs_out_attr3 = uniforms.f[93].yxxx;
return false;
}
bool sub_0() {
vs_out_attr2 = vs_in_reg2.xyzz;
sub_1();
return true;
}
// reference: B43E65F9DF0607E1, 707D664331545344
// shader: 8B30, 51E90820EFD2C2F1
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.ggg), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) - (vec3(1) - rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C40C846EE8567893, 51E90820EFD2C2F1
// program: 707D664331545344, 5840B6EDF6A7B4C9, 51E90820EFD2C2F1
// shader: 8B31, DD24330D867C1A2A

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp2.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp2.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp2.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp2.w = dot_s(uniforms.f[11], reg_tmp1);
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp2);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp2);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp2);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp2);
vs_out_attr1 = vs_in_reg1.xyyy;
return false;
}
bool sub_0() {
sub_1();
vs_out_attr2 = vs_in_reg2.xyzz;
return true;
}
// reference: 2889471B04EBE4F0, DD24330D867C1A2A
// shader: 8B30, DF7FB7D96A147766
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A0370CB1835CBC9C, DF7FB7D96A147766
// program: DD24330D867C1A2A, BF8D50647A6521FD, DF7FB7D96A147766
// shader: 8DD9, C54D64736CF1122F

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
struct Vertex {
    vec4 attributes[5];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(0.0, 0.0, 0.0, 0.0);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z);
    texcoord2 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[5](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0]);
    prim_buffer[1].attributes = vec4[5](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1]);
    prim_buffer[2].attributes = vec4[5](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 9B5D9FE61790B0B0, C54D64736CF1122F
// shader: 8B31, 3614B8D4AC41159A

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = reg_tmp0.xyyy;
vs_out_attr4 = reg_tmp0.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg4.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg4.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg4.xyz);
reg_tmp13.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp13.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp13.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
sub_2();
return false;
}
bool sub_2() {
uint jmp_to = 80u;
while (true) {
switch (jmp_to) {
case 80u:
vs_out_attr3 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 116u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 154u; break;
}
case 116u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_3();
} else {
sub_8();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 154u:
vs_out_attr2 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
if (bool_regs.y) {
sub_4();
} else {
sub_5();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_4() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_5() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_6();
} else {
sub_7();
}
return false;
}
bool sub_6() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_7() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_8() {
if (bool_regs.y) {
sub_9();
} else {
sub_10();
}
return false;
}
bool sub_9() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_0() {
reg_tmp0.x = (vs_in_reg3.wwww).x;
reg_tmp0.y = (vs_in_reg4.wwww).y;
sub_1();
return true;
}
// reference: F951F46584E03BB0, 3614B8D4AC41159A
// shader: 8B30, FD62DD43E04F8171
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(vec3(dot((vec3(1) - texcolor1.rgb) - vec3(0.5), (const_color[0].rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (vec3(1) - last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CEE0946BDA1056E0, FD62DD43E04F8171
// program: 3614B8D4AC41159A, C54D64736CF1122F, FD62DD43E04F8171
// reference: F951F465FB25EA28, 3614B8D4AC41159A
// shader: 8B30, 0E4C63B31F1A729D
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(vec3(dot((texcolor1.rgb) - vec3(0.5), (const_color[0].rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 190DB0FDDA1056E0, 0E4C63B31F1A729D
// program: 3614B8D4AC41159A, C54D64736CF1122F, 0E4C63B31F1A729D
// shader: 8B31, 2F89EB07A16A1197

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=5) in vec4 vs_in_reg5;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = vs_in_reg1.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp2.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp2.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp2.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp2.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp1.x = dot_s(uniforms.f[8], reg_tmp2);
reg_tmp1.y = dot_s(uniforms.f[9], reg_tmp2);
reg_tmp1.z = dot_s(uniforms.f[10], reg_tmp2);
reg_tmp1.w = dot_s(uniforms.f[11], reg_tmp2);
reg_tmp13.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp13.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp13.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp13.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp13.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp13.xyz);
vs_out_attr3 = -reg_tmp1;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp1);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp1);
bool_regs = equal(-uniforms.f[93].yy, reg_tmp14.zz);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp1);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp1);
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
vs_out_attr2.w = (uniforms.f[93].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (!bool_regs.x) {
sub_2();
} else {
sub_3();
}
return false;
}
bool sub_2() {
vs_out_attr2.z = rcp_s(reg_tmp4.x);
vs_out_attr2.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_3() {
vs_out_attr2 = uniforms.f[93].yxxx;
return false;
}
bool sub_0() {
vs_out_attr4 = vs_in_reg5.xyyy;
sub_1();
return true;
}
// reference: F951F46512B19F4D, 2F89EB07A16A1197
// shader: 8B30, BC55B0B5C300B020
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BB172454483A319F, BC55B0B5C300B020
// program: 2F89EB07A16A1197, C54D64736CF1122F, BC55B0B5C300B020
// shader: 8DD9, DB5E751A0F7B7CE3

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
struct Vertex {
    vec4 attributes[3];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(0.0, 0.0, 0.0, 0.0);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(0.0, 0.0, 0.0, 0.0);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);
    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[3](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0]);
    prim_buffer[1].attributes = vec4[3](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1]);
    prim_buffer[2].attributes = vec4[3](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: FDD242F383F0276A, DB5E751A0F7B7CE3
// shader: 8B31, E84BD3B74AD0E4B1

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp2.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp2.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp2.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp2.w = dot_s(uniforms.f[11], reg_tmp1);
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp2);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp2);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp2);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp2);
vs_out_attr1 = vs_in_reg1.xyyy;
return false;
}
bool sub_0() {
sub_1();
vs_out_attr2 = vs_in_reg1.xyyy;
return true;
}
// reference: 2889471BBA78049B, E84BD3B74AD0E4B1
// shader: 8B30, 4D39390953F67B3C
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 79C6E70D61D61444, 4D39390953F67B3C
// program: E84BD3B74AD0E4B1, DB5E751A0F7B7CE3, 4D39390953F67B3C
// shader: 8B30, 5A61C74DAAD2C134
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa) + (texture(tex_cube, vec3(texcoord0, texcoord0_w)).rgb) * (vec3(1) - (const_color[1].aaa)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1) - (vec3(1) - rounded_primary_color.aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (primary_fragment_color.aaa), vec3(1)) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B08F5B8C94A94800, 5A61C74DAAD2C134
// program: 2F6AB1438B14E98E, 0E1DAAC943BB278F, 5A61C74DAAD2C134
// shader: 8B30, 1FC99BADA9172040
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (vec3(1) - texcolor2.rrr) + (texture(tex_cube, vec3(texcoord0, texcoord0_w)).rgb) * (vec3(1) - (vec3(1) - texcolor2.rrr)), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor2.rrr), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A99C9B3A94D87557, 1FC99BADA9172040
// program: 1A640611D16050C2, 66C01D9968A40A5A, 1FC99BADA9172040
// shader: 8B30, F530D76D2A4F2162
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rrr) + (const_color[0].rrr) - vec3(0.5), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor1.rrr) * (const_color[1].rrr), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a < alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FF14047C2CDD77E5, F530D76D2A4F2162
// program: AB6AD9F21067B359, 3CDADE164F37EFAA, F530D76D2A4F2162
// shader: 8DD9, 63BB35208E172304

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
layout(location=5) in vec4 vs_out_attr5[];
struct Vertex {
    vec4 attributes[6];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(0.0, 0.0, 0.0, 0.0);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = vtx.attributes[5].z;
    view = vec3(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z);
    texcoord2 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[6](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0]);
    prim_buffer[1].attributes = vec4[6](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1]);
    prim_buffer[2].attributes = vec4[6](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: A3B58A5B10511CAC, 63BB35208E172304
// shader: 8B31, 97B893B3B485AB23

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_0();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0.x = (vs_in_reg3.wwww).x;
reg_tmp0.y = (vs_in_reg4.wwww).y;
sub_1();
return true;
}
bool sub_1() {
uint jmp_to = 258u;
while (true) {
switch (jmp_to) {
case 258u:
vs_out_attr1 = reg_tmp0.xyyy;
vs_out_attr4 = reg_tmp0.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg4.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg4.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg4.xyz);
reg_tmp13.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp13.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp13.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
vs_out_attr3 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 312u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
vs_out_attr2 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 348u; break;
}
case 312u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_2();
} else {
sub_7();
}
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
vs_out_attr2 = mul_s(reg_tmp8, reg_tmp6);
case 348u:
reg_tmp0 = vec4(dot_3(-reg_tmp15.xyz, reg_tmp14.xyz));
reg_tmp0 = reg_tmp0 + reg_tmp0;
reg_tmp0 = fma_s(reg_tmp0, reg_tmp14, reg_tmp15);
vs_out_attr5.x = (reg_tmp0.xxxx).x;
vs_out_attr5.y = (reg_tmp0.yyyy).y;
vs_out_attr5.z = (reg_tmp0.zzzz).z;
vs_out_attr5.w = (uniforms.f[93].xxxx).w;
default: return false;
}
}
return false;
}
bool sub_2() {
if (bool_regs.y) {
sub_3();
} else {
sub_4();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
return false;
}
bool sub_3() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_4() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_5();
} else {
sub_6();
}
return false;
}
bool sub_5() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_6() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_7() {
if (bool_regs.y) {
sub_8();
} else {
sub_9();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
return false;
}
bool sub_8() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_9() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
// reference: D3ED44ED773459A2, 97B893B3B485AB23
// shader: 8B30, BF9F5D97222B7C74
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (const_color[0].rgb) + (texture(tex_cube, vec3(texcoord0, texcoord0_w)).rgb) * (vec3(1) - (const_color[0].rgb)), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - texcolor2.bbb) + (texcolor1.rgb) * (vec3(1) - (vec3(1) - texcolor2.bbb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1C43F002AE4285F0, BF9F5D97222B7C74
// program: 97B893B3B485AB23, 63BB35208E172304, BF9F5D97222B7C74
// shader: 8B31, E76939E0BD478AAA

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=2) in vec4 vs_in_reg2;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = reg_tmp0.xyyy;
vs_out_attr5 = reg_tmp0.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg4.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg4.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg4.xyz);
reg_tmp13.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp13.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp13.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
sub_2();
return false;
}
bool sub_2() {
uint jmp_to = 80u;
while (true) {
switch (jmp_to) {
case 80u:
vs_out_attr4 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 116u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 154u; break;
}
case 116u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_3();
} else {
sub_8();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 154u:
vs_out_attr3 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
if (bool_regs.y) {
sub_4();
} else {
sub_5();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_4() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_5() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_6();
} else {
sub_7();
}
return false;
}
bool sub_6() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_7() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_8() {
if (bool_regs.y) {
sub_9();
} else {
sub_10();
}
return false;
}
bool sub_9() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_0() {
vs_out_attr2 = vs_in_reg2.xyzz;
reg_tmp0.x = (vs_in_reg3.wwww).x;
reg_tmp0.y = (vs_in_reg4.wwww).y;
sub_1();
return true;
}
// reference: 263EB2143997355C, E76939E0BD478AAA
// shader: 8B30, 1166ADD4D1C6D4A9
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb) + (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp(vec3(dot((last_tex_env_out.rgb) - vec3(0.5), (const_color[1].rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6C5E498339DB3C53, 1166ADD4D1C6D4A9
// program: E76939E0BD478AAA, CF9491352B19EDBB, 1166ADD4D1C6D4A9
// reference: 263EB214169DD5B3, E76939E0BD478AAA
// shader: 8B30, EB559D8D0D152773
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(vec3(dot((texcolor1.rgb) - vec3(0.5), (texcolor1.rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((secondary_fragment_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AFF47BE352EB7CA7, EB559D8D0D152773
// program: E76939E0BD478AAA, CF9491352B19EDBB, EB559D8D0D152773
// reference: 263EB214E27AD64D, E76939E0BD478AAA
// shader: 8B30, 26AACCC16FA33FB3
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor2.b);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.ggg), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) - (vec3(1) - rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a < alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 69BD1FBBC49E1AD8, 26AACCC16FA33FB3
// program: E76939E0BD478AAA, CF9491352B19EDBB, 26AACCC16FA33FB3
// reference: 263EB214742B72B0, E76939E0BD478AAA
// shader: 8B30, 9946AD9B28FBCEFA
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.ggg), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) - (vec3(1) - rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C40C846EC661083E, 9946AD9B28FBCEFA
// program: E76939E0BD478AAA, CF9491352B19EDBB, 9946AD9B28FBCEFA
// reference: B43E65F9B61A97E5, 707D664331545344
// shader: 8B30, 156A00E70BC4B92B
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.aaa) + (const_color[1].aaa) * (vec3(1) - (texcolor1.aaa)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a < alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B81744162EAC4168, 156A00E70BC4B92B
// program: 707D664331545344, 5840B6EDF6A7B4C9, 156A00E70BC4B92B
// reference: F951F465C230DC92, 3614B8D4AC41159A
// shader: 8B30, 1D84645B5243D776
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(vec3(dot((vec3(1) - texcolor1.rgb) - vec3(0.5), (const_color[0].rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor2.b);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (vec3(1) - last_tex_env_out.rrr), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a < alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3B458F8A2D0BC883, 1D84645B5243D776
// program: 3614B8D4AC41159A, C54D64736CF1122F, 1D84645B5243D776
// shader: 8B31, 8F8E6D9A036D7DD6

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg4.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg4.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg4.xyz);
reg_tmp13.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp13.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp13.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
sub_2();
return false;
}
bool sub_2() {
uint jmp_to = 80u;
while (true) {
switch (jmp_to) {
case 80u:
vs_out_attr3 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 116u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 154u; break;
}
case 116u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_3();
} else {
sub_8();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 154u:
vs_out_attr2 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
if (bool_regs.y) {
sub_4();
} else {
sub_5();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_4() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_5() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_6();
} else {
sub_7();
}
return false;
}
bool sub_6() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_7() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_8() {
if (bool_regs.y) {
sub_9();
} else {
sub_10();
}
return false;
}
bool sub_9() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_0() {
reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
reg_tmp1.xy = (vs_in_reg1.xyyy).xy;
reg_tmp0.x = (uniforms.f[12].xxxx + reg_tmp0.xxxx).x;
reg_tmp1.x = (uniforms.f[12].xxxx + reg_tmp1.xxxx).x;
reg_tmp1.y = (uniforms.f[12].yyyy + reg_tmp1.yyyy).y;
vs_out_attr1 = reg_tmp0;
vs_out_attr4 = reg_tmp1;
sub_1();
return true;
}
// reference: F951F4657CA33CF9, 8F8E6D9A036D7DD6
// reference: CEE0946B2FF4DA65, FD62DD43E04F8171
// program: 8F8E6D9A036D7DD6, C54D64736CF1122F, FD62DD43E04F8171
// shader: 8B31, C4C0510353E32A24

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = reg_tmp0.xyyy;
vs_out_attr4 = reg_tmp0.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp15.w = dot_s(uniforms.f[11], reg_tmp1);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
reg_tmp0.x = dot_3(uniforms.f[4].xyz, vs_in_reg4.xyz);
reg_tmp0.y = dot_3(uniforms.f[5].xyz, vs_in_reg4.xyz);
reg_tmp0.z = dot_3(uniforms.f[6].xyz, vs_in_reg4.xyz);
reg_tmp13.x = dot_3(uniforms.f[8].xyz, reg_tmp0.xyz);
reg_tmp13.y = dot_3(uniforms.f[9].xyz, reg_tmp0.xyz);
reg_tmp13.z = dot_3(uniforms.f[10].xyz, reg_tmp0.xyz);
sub_2();
return false;
}
bool sub_2() {
uint jmp_to = 80u;
while (true) {
switch (jmp_to) {
case 80u:
vs_out_attr3 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 116u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 154u; break;
}
case 116u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_3();
} else {
sub_8();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 154u:
vs_out_attr2 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
if (bool_regs.y) {
sub_4();
} else {
sub_5();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_4() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_5() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_6();
} else {
sub_7();
}
return false;
}
bool sub_6() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_7() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_8() {
if (bool_regs.y) {
sub_9();
} else {
sub_10();
}
return false;
}
bool sub_9() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_0() {
reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
reg_tmp0.y = (uniforms.f[12].yyyy + reg_tmp0.yyyy).y;
vs_out_attr5 = reg_tmp0.xyyy;
reg_tmp0 = vs_in_reg1;
sub_1();
return true;
}
// reference: D3ED44ED4864E4F3, C4C0510353E32A24
// shader: 8B30, 8F96A0076EE6057A
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor2).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.rrr), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6A84CAA4CF4181F1, 8F96A0076EE6057A
// program: C4C0510353E32A24, 410E4C5F77308E32, 8F96A0076EE6057A
// shader: 8B31, AF40B657E4F21ADF

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp1.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp2.x = dot_s(uniforms.f[8], reg_tmp1);
reg_tmp2.y = dot_s(uniforms.f[9], reg_tmp1);
reg_tmp2.z = dot_s(uniforms.f[10], reg_tmp1);
reg_tmp2.w = dot_s(uniforms.f[11], reg_tmp1);
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp2);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp2);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp2);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp2);
vs_out_attr1 = vs_in_reg1.xyyy;
return false;
}
bool sub_0() {
sub_1();
vs_out_attr2 = vs_in_reg2;
return true;
}
// reference: 2889471BEDB9065C, AF40B657E4F21ADF
// shader: 8B30, F12A604DDC5531BF
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CE839A23B7283880, F12A604DDC5531BF
// program: AF40B657E4F21ADF, BF8D50647A6521FD, F12A604DDC5531BF
// shader: 8DD9, B645B460C646836D

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
struct Vertex {
    vec4 attributes[5];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(0.0, 0.0, 0.0, 0.0);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z, vtx.attributes[2].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);
    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[5](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0]);
    prim_buffer[1].attributes = vec4[5](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1]);
    prim_buffer[2].attributes = vec4[5](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 5AAA17FA60082BE2, B645B460C646836D
// shader: 8B30, 4BE45EE426FE24E8
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.ggg), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) - (vec3(1) - rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 956E26514BC9F3ED, 4BE45EE426FE24E8
// program: 707D664331545344, B645B460C646836D, 4BE45EE426FE24E8
// shader: 8B30, C3D1BC9580C38879
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CE839A23835CBC9C, C3D1BC9580C38879
// program: DD24330D867C1A2A, BF8D50647A6521FD, C3D1BC9580C38879
// reference: C40C846E1DB2F416, 51E90820EFD2C2F1
// reference: CE839A2342CCB405, F12A604DDC5531BF
// reference: 956E2651BE2D7F68, 4BE45EE426FE24E8
// reference: CE839A2376B83019, C3D1BC9580C38879
// reference: 190DB0FD2FF4DA65, 0E4C63B31F1A729D
// reference: BB172454BDDEBD1A, BC55B0B5C300B020
// reference: F42CAA70707C4823, F77AAEE911D965A7
// reference: A0370CB176B83019, DF7FB7D96A147766
// reference: B43E65F93D384583, 707D664331545344
// shader: 8B30, AFBC9E33A661C3E9
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (primary_fragment_color.rgb);
float alpha_output_0 = byteround(clamp((texcolor1.r) * (rounded_primary_color.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((texcolor1.g) * (rounded_primary_color.g) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((texcolor1.a) * (rounded_primary_color.b) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 40DFABC1CB263A23, AFBC9E33A661C3E9
// program: 707D664331545344, 5840B6EDF6A7B4C9, AFBC9E33A661C3E9
// reference: 2889471B569CB650, AF40B657E4F21ADF
// reference: 2889471B861DF58F, AF40B657E4F21ADF
// shader: 8B30, 7F84258D4FB0FA1D
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.aaa) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2B7A73BD42CCB405, 7F84258D4FB0FA1D
// program: AF40B657E4F21ADF, BF8D50647A6521FD, 7F84258D4FB0FA1D
// shader: 8B31, 930E3B305636024B

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;
layout(location=3) in vec4 vs_in_reg3;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
vs_out_attr1 = vs_in_reg1.xyyy;
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[92]).w;
reg_tmp2.x = dot_s(uniforms.f[4], reg_tmp0);
reg_tmp2.y = dot_s(uniforms.f[5], reg_tmp0);
reg_tmp2.z = dot_s(uniforms.f[6], reg_tmp0);
reg_tmp2.w = dot_s(uniforms.f[7], reg_tmp0);
reg_tmp1.x = dot_s(uniforms.f[8], reg_tmp2);
reg_tmp1.y = dot_s(uniforms.f[9], reg_tmp2);
reg_tmp1.z = dot_s(uniforms.f[10], reg_tmp2);
reg_tmp1.w = dot_s(uniforms.f[11], reg_tmp2);
reg_tmp13.x = dot_3(uniforms.f[4].xyz, vs_in_reg3.xyz);
reg_tmp13.y = dot_3(uniforms.f[5].xyz, vs_in_reg3.xyz);
reg_tmp13.z = dot_3(uniforms.f[6].xyz, vs_in_reg3.xyz);
reg_tmp14.x = dot_3(uniforms.f[8].xyz, reg_tmp13.xyz);
reg_tmp14.y = dot_3(uniforms.f[9].xyz, reg_tmp13.xyz);
reg_tmp14.z = dot_3(uniforms.f[10].xyz, reg_tmp13.xyz);
vs_out_attr4 = -reg_tmp1;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp1);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp1);
bool_regs = equal(-uniforms.f[93].yy, reg_tmp14.zz);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp1);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp1);
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
vs_out_attr3.w = (uniforms.f[93].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (!bool_regs.x) {
sub_2();
} else {
sub_3();
}
return false;
}
bool sub_2() {
vs_out_attr3.z = rcp_s(reg_tmp4.x);
vs_out_attr3.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_3() {
vs_out_attr3 = uniforms.f[93].yxxx;
return false;
}
bool sub_0() {
vs_out_attr5 = vs_in_reg1.xyzz;
vs_out_attr2 = vs_in_reg2.xyzz;
sub_1();
return true;
}
// reference: 263EB214378B523C, 930E3B305636024B
// shader: 8B30, 2C58000D889AB23B
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}
float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}
float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor2.b);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.ggg), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) - (vec3(1) - rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a < alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 69BD1FBB1F4DE6F0, 2C58000D889AB23B
// program: 930E3B305636024B, CF9491352B19EDBB, 2C58000D889AB23B

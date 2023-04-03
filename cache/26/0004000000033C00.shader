// shader: 8DD9, 422B94F1339C42ED

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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(0.0, 0.0);

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
// reference: 6D98C2C476DC3F58, 422B94F1339C42ED
// shader: 8B31, 6B584E1C08F380DD

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
vs_out_attr0.xy = (vs_in_reg0.xyyy).xy;
vs_out_attr0.z = (-uniforms.f[5].wwww).z;
vs_out_attr0.w = (uniforms.f[95].wwww).w;
vs_out_attr1 = uniforms.f[6].wzyx;
return true;
}
// reference: E07DBC51294BAAD9, 6B584E1C08F380DD
// shader: 8B30, 887808CD0BBA6904
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C56D9496B367F46D, 887808CD0BBA6904
// program: 6B584E1C08F380DD, 422B94F1339C42ED, 887808CD0BBA6904
// shader: 8DD9, 80D245930F7B7CE3

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

    texcoord0 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);
    texcoord1 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);

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
// reference: 91BCC71FEF9EA286, 80D245930F7B7CE3
// shader: 8B31, 3497E0596B34DC9E

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
vs_out_attr0.xy = (vs_in_reg0.xyyy).xy;
vs_out_attr0.z = (uniforms.f[95].wwww).z;
vs_out_attr0.w = (uniforms.f[95].zzzz).w;
vs_out_attr1 = mul_s(uniforms.f[5].wzyx, vs_in_reg0.zwzw);
vs_out_attr2 = vs_in_reg0.zwzw;
return true;
}
// reference: 8972D72140453EB9, 3497E0596B34DC9E
// shader: 8B30, 45E2174AE15FBF0E
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C56D94960BC51C0D, 45E2174AE15FBF0E
// program: 3497E0596B34DC9E, 80D245930F7B7CE3, 45E2174AE15FBF0E
// shader: 8DD9, DC0A6D35EDF1ED6F

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)
layout(points) in;
layout(triangle_strip, max_vertices=30) out;
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
layout(location=7) in vec4 vs_out_attr7[];
layout(location=8) in vec4 vs_out_attr8[];
layout(location=9) in vec4 vs_out_attr9[];

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms gs_uniforms
layout(binding=3, std140) uniform gs_config {
    pica_uniforms uniforms;
};
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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);
    texcoord1 = vec2(0.0, 0.0);

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

Vertex output_buffer;
Vertex prim_buffer[3];
uint vertex_id = 0u;
bool prim_emit = false;
bool winding = false;
void setemit(uint vertex_id_, bool prim_emit_, bool winding_);
void emit();
void main() {
    output_buffer.attributes[0] = vec4(0.0, 0.0, 0.0, 1.0);
    output_buffer.attributes[1] = vec4(0.0, 0.0, 0.0, 1.0);
    output_buffer.attributes[2] = vec4(0.0, 0.0, 0.0, 1.0);
    exec_shader();
}

void setemit(uint vertex_id_, bool prim_emit_, bool winding_) {
    vertex_id = vertex_id_;
    prim_emit = prim_emit_;
    winding = winding_;
}
void emit() {
    prim_buffer[vertex_id] = output_buffer;
    if (prim_emit) {
        if (winding) {
            EmitPrim(prim_buffer[1], prim_buffer[0], prim_buffer[2]);
            winding = false;
        } else {
            EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
        }
    }
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

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
bool_regs = equal(uniforms.f[80].xy, vs_out_attr6[0].xy);
reg_tmp4.xy = (vs_out_attr0[0].xyyy).xy;
reg_tmp5.xy = (vs_out_attr0[0].zyyy).xy;
reg_tmp6.xy = (vs_out_attr0[0].xwww).xy;
reg_tmp7.xy = (vs_out_attr0[0].zwww).xy;
reg_tmp4.zw = (uniforms.f[80].xyxy).zw;
reg_tmp5.zw = (uniforms.f[80].xyxy).zw;
reg_tmp6.zw = (uniforms.f[80].xyxy).zw;
reg_tmp7.zw = (uniforms.f[80].xyxy).zw;
reg_tmp0.x = dot_s(reg_tmp4, vs_out_attr7[0]);
reg_tmp1.x = dot_s(reg_tmp5, vs_out_attr7[0]);
reg_tmp2.x = dot_s(reg_tmp6, vs_out_attr7[0]);
reg_tmp3.x = dot_s(reg_tmp7, vs_out_attr7[0]);
reg_tmp0.y = dot_s(reg_tmp4, vs_out_attr8[0]);
reg_tmp1.y = dot_s(reg_tmp5, vs_out_attr8[0]);
reg_tmp2.y = dot_s(reg_tmp6, vs_out_attr8[0]);
reg_tmp3.y = dot_s(reg_tmp7, vs_out_attr8[0]);
reg_tmp4.xy = (mul_s(reg_tmp0.xyyy, vs_out_attr9[0].xyyy)).xy;
reg_tmp5.xy = (mul_s(reg_tmp1.xyyy, vs_out_attr9[0].xyyy)).xy;
reg_tmp6.xy = (mul_s(reg_tmp2.xyyy, vs_out_attr9[0].xyyy)).xy;
reg_tmp7.xy = (mul_s(reg_tmp3.xyyy, vs_out_attr9[0].xyyy)).xy;
reg_tmp4.y = (reg_tmp4.yyyy + vs_out_attr6[0].yyyy).y;
reg_tmp5.y = (reg_tmp5.yyyy + vs_out_attr6[0].yyyy).y;
reg_tmp6.y = (reg_tmp6.yyyy + vs_out_attr6[0].yyyy).y;
reg_tmp7.y = (reg_tmp7.yyyy + vs_out_attr6[0].yyyy).y;
if (!bool_regs.x) {
sub_1();
} else {
sub_2();
}
return true;
}
bool sub_1() {
output_buffer.attributes[0].xy = (reg_tmp4.xyyy + vs_out_attr9[0].zwww).xy;
output_buffer.attributes[0].z = (vs_out_attr6[0].zzzz).z;
output_buffer.attributes[0].w = (uniforms.f[80].yyyy).w;
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (vs_out_attr5[0].xyyy).xy;
setemit(0u, false, false);
emit();
output_buffer.attributes[0].xy = (reg_tmp5.xyyy + vs_out_attr9[0].zwww).xy;
output_buffer.attributes[0].z = (vs_out_attr6[0].zzzz).z;
output_buffer.attributes[0].w = (uniforms.f[80].yyyy).w;
output_buffer.attributes[1] = vs_out_attr2[0];
output_buffer.attributes[2].xy = (vs_out_attr5[0].zyyy).xy;
setemit(1u, false, false);
emit();
output_buffer.attributes[0].xy = (reg_tmp6.xyyy + vs_out_attr9[0].zwww).xy;
output_buffer.attributes[0].z = (vs_out_attr6[0].zzzz).z;
output_buffer.attributes[0].w = (uniforms.f[80].yyyy).w;
output_buffer.attributes[1] = vs_out_attr3[0];
output_buffer.attributes[2].xy = (vs_out_attr5[0].xwww).xy;
setemit(2u, true, false);
emit();
output_buffer.attributes[0].xy = (reg_tmp7.xyyy + vs_out_attr9[0].zwww).xy;
output_buffer.attributes[0].z = (vs_out_attr6[0].zzzz).z;
output_buffer.attributes[0].w = (uniforms.f[80].yyyy).w;
output_buffer.attributes[1] = vs_out_attr4[0];
output_buffer.attributes[2].xy = (vs_out_attr5[0].zwww).xy;
setemit(0u, true, true);
emit();
return false;
}
bool sub_2() {
output_buffer.attributes[0].xy = (reg_tmp5.xyyy + vs_out_attr9[0].zwww).xy;
output_buffer.attributes[0].z = (vs_out_attr6[0].zzzz).z;
output_buffer.attributes[0].w = (uniforms.f[80].yyyy).w;
output_buffer.attributes[1] = vs_out_attr2[0];
output_buffer.attributes[2].xy = (vs_out_attr5[0].zyyy).xy;
setemit(0u, false, false);
emit();
output_buffer.attributes[0].xy = (reg_tmp4.xyyy + vs_out_attr9[0].zwww).xy;
output_buffer.attributes[0].z = (vs_out_attr6[0].zzzz).z;
output_buffer.attributes[0].w = (uniforms.f[80].yyyy).w;
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (vs_out_attr5[0].xyyy).xy;
setemit(1u, false, false);
emit();
output_buffer.attributes[0].xy = (reg_tmp7.xyyy + vs_out_attr9[0].zwww).xy;
output_buffer.attributes[0].z = (vs_out_attr6[0].zzzz).z;
output_buffer.attributes[0].w = (uniforms.f[80].yyyy).w;
output_buffer.attributes[1] = vs_out_attr4[0];
output_buffer.attributes[2].xy = (vs_out_attr5[0].zwww).xy;
setemit(2u, true, false);
emit();
output_buffer.attributes[0].xy = (reg_tmp6.xyyy + vs_out_attr9[0].zwww).xy;
output_buffer.attributes[0].z = (vs_out_attr6[0].zzzz).z;
output_buffer.attributes[0].w = (uniforms.f[80].yyyy).w;
output_buffer.attributes[1] = vs_out_attr3[0];
output_buffer.attributes[2].xy = (vs_out_attr5[0].xwww).xy;
setemit(0u, true, true);
emit();
return false;
}
// reference: EC7DCB15306E15FC, DC0A6D35EDF1ED6F
// shader: 8B31, 0D2DBB6B15798A3B

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
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;
layout(location=7) out vec4 vs_out_attr7;
layout(location=8) out vec4 vs_out_attr8;
layout(location=9) out vec4 vs_out_attr9;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
vs_out_attr7 = vec4(0, 0, 0, 1);
vs_out_attr8 = vec4(0, 0, 0, 1);
vs_out_attr9 = vec4(0, 0, 0, 1);
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

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
vs_out_attr0 = vs_in_reg0;
vs_out_attr1 = mul_s(uniforms.f[95].wwww, vs_in_reg1);
vs_out_attr2 = mul_s(uniforms.f[95].wwww, vs_in_reg2);
vs_out_attr3 = mul_s(uniforms.f[95].wwww, vs_in_reg3);
vs_out_attr4 = mul_s(uniforms.f[95].wwww, vs_in_reg4);
reg_tmp0 = mul_s(uniforms.f[5].wzwz, vs_in_reg5);
vs_out_attr5.xz = (reg_tmp0.xzzz).xz;
vs_out_attr5.yw = (uniforms.f[90].zzzz + -reg_tmp0.yyww).yw;
vs_out_attr6.x = (vs_in_reg6.xxxx).x;
if (uniforms.b[0]) {
sub_1();
} else {
sub_6();
}
reg_tmp1.z = (uniforms.f[6].yyyy).z;
reg_tmp0.z = (min_s(-uniforms.f[95].yyyy, reg_tmp1.zzzz)).z;
vs_out_attr6.z = (max_s(-uniforms.f[95].xxxx, reg_tmp0.zzzz)).z;
vs_out_attr6.w = (uniforms.f[95].xxxx).w;
vs_out_attr7 = uniforms.f[7].wzyx;
vs_out_attr8 = uniforms.f[8].wzyx;
vs_out_attr9 = uniforms.f[11].wzyx;
return true;
}
bool sub_1() {
reg_tmp4 = uniforms.f[90].wzyx;
bool_regs = equal(uniforms.f[4].wz, reg_tmp4.xy);
if (bool_regs.x) {
sub_2();
} else {
sub_3();
}
return false;
}
bool sub_2() {
vs_out_attr6.y = (uniforms.f[95].zzzz).y;
return false;
}
bool sub_3() {
if (!bool_regs.y) {
sub_4();
} else {
sub_5();
}
return false;
}
bool sub_4() {
vs_out_attr6.y = (-uniforms.f[6].zzzz).y;
return false;
}
bool sub_5() {
vs_out_attr6.y = (uniforms.f[6].zzzz).y;
return false;
}
bool sub_6() {
vs_out_attr6.y = (uniforms.f[95].zzzz).y;
return false;
}
// reference: 93B796209FD25144, 0D2DBB6B15798A3B
// shader: 8B30, 1EE4DB54AED21359
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) + (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2FCA965081BA9229, 1EE4DB54AED21359
// program: 0D2DBB6B15798A3B, DC0A6D35EDF1ED6F, 1EE4DB54AED21359
// shader: 8B30, CBB47822AB5E1753
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) + (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E062DF451B0E0DFE, CBB47822AB5E1753
// program: 0D2DBB6B15798A3B, DC0A6D35EDF1ED6F, CBB47822AB5E1753
// shader: 8DD9, 6F6B8D04DE5400F0

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

    texcoord0 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);
    texcoord1 = vec2(0.0, 0.0);

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
// reference: E0A3E62E72D05152, 6F6B8D04DE5400F0
// shader: 8B31, 24F1A02FAD844B71

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
vs_out_attr0.xy = (vs_in_reg0.xyyy).xy;
vs_out_attr0.zw = (uniforms.f[90].wzwz).zw;
reg_tmp0 = vs_in_reg0.zwzw;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
reg_tmp1.x = dot_s(uniforms.f[5].wzyx, reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[6].wzyx, reg_tmp0);
vs_out_attr1 = reg_tmp1.xyxy;
return true;
}
// reference: E11E134EB4F347AF, 24F1A02FAD844B71
// shader: 8B30, DDA61FA7AF3B4489
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 201D917E00EF9A82, DDA61FA7AF3B4489
// program: 24F1A02FAD844B71, 6F6B8D04DE5400F0, DDA61FA7AF3B4489
// shader: 8B30, 4C9684C02DD9FF08
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C6515778C3B71EAB, 4C9684C02DD9FF08
// program: 0D2DBB6B15798A3B, DC0A6D35EDF1ED6F, 4C9684C02DD9FF08
// shader: 8DD9, DADA7DA78218FFEA

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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);
    texcoord1 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);

    texcoord0_w = vtx.attributes[5].z;
    view = vec3(vtx.attributes[4].x, vtx.attributes[4].y, vtx.attributes[4].z);
    texcoord2 = vec2(vtx.attributes[2].z, vtx.attributes[2].w);

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
// reference: 46D4F628ABD6EA28, DADA7DA78218FFEA
// shader: 8B31, 32B606A8D81B7AB9

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

bool sub_0();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_20();
bool sub_6();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp1 = vs_in_reg0.xyzz;
reg_tmp1.w = (uniforms.f[95].wwww).w;
reg_tmp8.x = dot_s(uniforms.f[5].wzyx, reg_tmp1);
reg_tmp8.y = dot_s(uniforms.f[6].wzyx, reg_tmp1);
reg_tmp8.z = dot_s(uniforms.f[7].wzyx, reg_tmp1);
reg_tmp8.w = (uniforms.f[95].wwww).w;
reg_tmp5 = vs_in_reg1.xyzz;
reg_tmp7.x = dot_3(uniforms.f[5].wzy, reg_tmp5.xyz);
reg_tmp7.y = dot_3(uniforms.f[6].wzy, reg_tmp5.xyz);
reg_tmp7.z = dot_3(uniforms.f[7].wzy, reg_tmp5.xyz);
reg_tmp7.w = (uniforms.f[95].zzzz).w;
if (uniforms.b[0]) {
sub_1();
}
vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp8);
vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp8);
vs_out_attr0.z = dot_s(-uniforms.f[2].wzyx, reg_tmp8);
vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp8);
reg_tmp2.x = dot_3(uniforms.f[24].wzy, reg_tmp7.xyz);
reg_tmp2.y = dot_3(uniforms.f[25].wzy, reg_tmp7.xyz);
reg_tmp2.z = dot_3(uniforms.f[26].wzy, reg_tmp7.xyz);
reg_tmp2.w = (uniforms.f[95].zzzz).w;
reg_tmp6.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp6 = mul_s(reg_tmp2, reg_tmp6.xxxx);
if (uniforms.b[8]) {
sub_9();
} else {
sub_10();
}
if (uniforms.b[4]) {
sub_13();
} else {
sub_14();
}
vs_out_attr5 = reg_tmp7.xyzz;
reg_tmp0.w = (mul_s(uniforms.f[95].xxxx, vs_in_reg1.wwww)).w;
reg_tmp11 = vs_in_reg2.xyxy;
if (uniforms.b[9]) {
sub_15();
} else {
sub_17();
}
reg_tmp10.x = dot_s(uniforms.f[24].wzyx, reg_tmp8);
reg_tmp10.y = dot_s(uniforms.f[25].wzyx, reg_tmp8);
reg_tmp10.z = dot_s(uniforms.f[26].wzyx, reg_tmp8);
reg_tmp10.w = dot_s(uniforms.f[27].wzyx, reg_tmp8);
reg_tmp3 = uniforms.f[41].wzyx;
reg_tmp4 = fma_s(reg_tmp11, uniforms.f[42].wzyx, reg_tmp3);
if (uniforms.b[6]) {
sub_20();
}
vs_out_attr2 = reg_tmp4;
vs_out_attr4 = -reg_tmp10;
vs_out_attr1 = reg_tmp0;
return true;
}
bool sub_1() {
if (uniforms.b[1]) {
sub_2();
} else {
sub_3();
}
reg_tmp4 = uniforms.f[95].zzzz;
if (uniforms.b[2]) {
sub_4();
}
if (uniforms.b[3]) {
sub_7();
}
reg_tmp8.y = (reg_tmp8.yyyy + reg_tmp4.yyyy).y;
return false;
}
bool sub_2() {
reg_tmp6 = reg_tmp8;
return false;
}
bool sub_3() {
reg_tmp5 = uniforms.f[95].zzzz;
reg_tmp6.x = dot_s(uniforms.f[5].wzyx, reg_tmp5);
reg_tmp6.y = dot_s(uniforms.f[6].wzyx, reg_tmp5);
return false;
}
bool sub_4() {
addr_regs.z = int(uniforms.i[1].y);
for (uint i = 0u; i <= uniforms.i[1].x; addr_regs.z += int(uniforms.i[1].z), ++i) {
sub_5();
}
return false;
}
bool sub_5() {
reg_tmp3.x = dot_3(uniforms.f[8 + addr_regs.z].wzy, reg_tmp6.xyz);
reg_tmp3.yz = (uniforms.f[16 + addr_regs.z].wzyx).yz;
reg_tmp3.x = (fma_s(reg_tmp3.xxxx, reg_tmp3.yyyy, reg_tmp3.zzzz)).x;
sub_6();
reg_tmp4 = fma_s(reg_tmp2.xyxx, uniforms.f[16 + addr_regs.z].wwww, reg_tmp4);
return false;
}
bool sub_7() {
addr_regs.z = int(uniforms.i[2].y);
for (uint i = 0u; i <= uniforms.i[2].x; addr_regs.z += int(uniforms.i[2].z), ++i) {
sub_8();
}
return false;
}
bool sub_8() {
reg_tmp10 = -uniforms.f[8 + addr_regs.z].wzyx + reg_tmp6;
reg_tmp11.x = dot_3(reg_tmp10.xyz, reg_tmp10.xyz);
reg_tmp11.x = rsq_s(reg_tmp11.x);
reg_tmp10.w = rcp_s(reg_tmp11.x);
reg_tmp10.xyz = (mul_s(reg_tmp10, reg_tmp11.xxxx)).xyz;
reg_tmp3.yz = (uniforms.f[16 + addr_regs.z].wzyx).yz;
reg_tmp3.x = (fma_s(reg_tmp10.wwww, reg_tmp3.yyyy, reg_tmp3.zzzz)).x;
sub_6();
reg_tmp3.x = (mul_s(-uniforms.f[8 + addr_regs.z].xxxx, reg_tmp10.wwww)).x;
reg_tmp3.x = (uniforms.f[95].wwww + reg_tmp3.xxxx).x;
reg_tmp3.x = (min_s(uniforms.f[95].wwww, reg_tmp3.xxxx)).x;
reg_tmp3.x = (max_s(uniforms.f[95].zzzz, reg_tmp3.xxxx)).x;
reg_tmp11.x = (mul_s(uniforms.f[16 + addr_regs.z].wwww, reg_tmp3.xxxx)).x;
reg_tmp3.x = (mul_s(reg_tmp3.xxxx, reg_tmp3.xxxx)).x;
reg_tmp10.xyz = (reg_tmp10.xyzz + -reg_tmp2.xyxx).xyz;
reg_tmp3.xyz = (fma_s(reg_tmp3.xxxx, reg_tmp10.xyzz, reg_tmp2.xyxx)).xyz;
reg_tmp4 = fma_s(reg_tmp3, reg_tmp11.xxxx, reg_tmp4);
return false;
}
bool sub_9() {
reg_tmp9 = vs_in_reg4.xyzz;
reg_tmp2.x = dot_3(uniforms.f[24].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[25].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[26].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[95].zzzz).w;
reg_tmp9.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp9.x = rsq_s(reg_tmp9.x);
reg_tmp9 = mul_s(reg_tmp2, reg_tmp9.xxxx);
reg_tmp9.w = (uniforms.f[95].wwww).w;
reg_tmp2 = vec4(lessThan(vs_in_reg4.wwww, -vs_in_reg4.wwww));
reg_tmp3 = vec4(lessThan(-vs_in_reg4.wwww, vs_in_reg4.wwww));
reg_tmp0 = reg_tmp3 + -reg_tmp2;
reg_tmp7 = mul_s(reg_tmp6.yzxx, reg_tmp9.zxyy);
reg_tmp7 = fma_s(-reg_tmp9.yzxx, reg_tmp6.zxyy, reg_tmp7);
reg_tmp7.w = dot_3(reg_tmp7.xyz, reg_tmp7.xyz);
reg_tmp7.w = rsq_s(reg_tmp7.w);
reg_tmp7 = mul_s(reg_tmp7, reg_tmp7.wwww);
reg_tmp7 = mul_s(reg_tmp7, reg_tmp0);
reg_tmp11.w = (reg_tmp6.zzzz + reg_tmp7.yyyy).w;
reg_tmp9 = mul_s(reg_tmp7.yzxx, reg_tmp6.zxyy);
reg_tmp9 = fma_s(-reg_tmp6.yzxx, reg_tmp7.zxyy, reg_tmp9);
reg_tmp11.w = (reg_tmp9.xxxx + reg_tmp11).w;
reg_tmp9.w = (reg_tmp7.zzzz).w;
reg_tmp7.z = (reg_tmp9.xxxx).z;
reg_tmp11.w = (uniforms.f[95].wwww + reg_tmp11).w;
reg_tmp6.w = (reg_tmp7.xxxx).w;
reg_tmp7.x = (reg_tmp6.zzzz).x;
reg_tmp11.x = (uniforms.f[95].wwww).x;
reg_tmp11.y = (-uniforms.f[95].wwww).y;
reg_tmp7.xz = (reg_tmp9.wwyy + -reg_tmp6.yyww).xz;
reg_tmp7.y = (reg_tmp6.xxxx + -reg_tmp9.zzzz).y;
reg_tmp7.w = (reg_tmp11).w;
reg_tmp11 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp11 = vec4(rsq_s(reg_tmp11.x));
vs_out_attr3 = mul_s(reg_tmp7, reg_tmp11);
return false;
}
bool sub_10() {
bool_regs = equal(-uniforms.f[95].ww, reg_tmp6.zz);
reg_tmp4 = uniforms.f[95].wwww + reg_tmp6.zzzz;
reg_tmp4 = mul_s(uniforms.f[95].yyyy, reg_tmp4);
vs_out_attr3.w = (uniforms.f[95].zzzz).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[95].yyyy, reg_tmp6);
if (!bool_regs.x) {
sub_11();
} else {
sub_12();
}
return false;
}
bool sub_11() {
vs_out_attr3.z = rcp_s(reg_tmp4.x);
vs_out_attr3.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_12() {
vs_out_attr3.x = (uniforms.f[95].wwww).x;
vs_out_attr3.yzw = (uniforms.f[95].zzzz).yzw;
return false;
}
bool sub_13() {
reg_tmp4 = -uniforms.f[28].wzyx + reg_tmp8;
reg_tmp4.w = (uniforms.f[95].zzzz).w;
reg_tmp5.x = dot_s(reg_tmp4, reg_tmp4);
reg_tmp5.x = rsq_s(reg_tmp5.x);
reg_tmp5 = mul_s(reg_tmp4, reg_tmp5.xxxx);
reg_tmp7.x = dot_s(uniforms.f[29].wzyx, reg_tmp5);
reg_tmp7.y = dot_s(uniforms.f[30].wzyx, reg_tmp5);
reg_tmp7.z = dot_s(uniforms.f[31].wzyx, reg_tmp5);
return false;
}
bool sub_14() {
reg_tmp7.x = dot_s(uniforms.f[29].wzyx, reg_tmp8);
reg_tmp7.y = dot_s(uniforms.f[30].wzyx, reg_tmp8);
reg_tmp7.z = dot_s(uniforms.f[31].wzyx, reg_tmp8);
return false;
}
bool sub_15() {
reg_tmp5 = mul_s(uniforms.f[94].wwww, vs_in_reg3);
reg_tmp1 = uniforms.f[94].zzzz + -reg_tmp5;
reg_tmp2.x = rcp_s(reg_tmp1.x);
reg_tmp2.y = rcp_s(reg_tmp1.y);
reg_tmp2.z = rcp_s(reg_tmp1.z);
reg_tmp5.xyz = (mul_s(reg_tmp5, reg_tmp2)).xyz;
reg_tmp0.xyz = (mul_s(uniforms.f[32].wzyx, reg_tmp5.xyzz)).xyz;
if (uniforms.b[5]) {
sub_16();
}
return false;
}
bool sub_16() {
reg_tmp0.w = (reg_tmp5.wwww).w;
return false;
}
bool sub_17() {
if (uniforms.b[10]) {
sub_18();
} else {
sub_19();
}
return false;
}
bool sub_18() {
reg_tmp11.zw = (mul_s(uniforms.f[94].yyyy, vs_in_reg3.xyxy)).zw;
reg_tmp0.xyz = (uniforms.f[33].wzyx).xyz;
return false;
}
bool sub_19() {
reg_tmp10 = mul_s(reg_tmp6, reg_tmp6);
reg_tmp5 = mul_s(reg_tmp6.xyzz, reg_tmp6.yzzx);
reg_tmp7 = reg_tmp6;
reg_tmp7.w = (uniforms.f[95].wwww).w;
reg_tmp14 = reg_tmp10.xxxx + -reg_tmp10.yyyy;
reg_tmp1.x = dot_s(uniforms.f[34].wzyx, reg_tmp7);
reg_tmp1.y = dot_s(uniforms.f[35].wzyx, reg_tmp7);
reg_tmp1.z = dot_s(uniforms.f[36].wzyx, reg_tmp7);
reg_tmp2.x = dot_s(uniforms.f[37].wzyx, reg_tmp5);
reg_tmp2.y = dot_s(uniforms.f[38].wzyx, reg_tmp5);
reg_tmp2.z = dot_s(uniforms.f[39].wzyx, reg_tmp5);
reg_tmp7 = fma_s(reg_tmp14.xxxx, uniforms.f[40].wzyx, reg_tmp1);
reg_tmp0.xyz = (reg_tmp7 + reg_tmp2).xyz;
reg_tmp0.xyz = (mul_s(uniforms.f[95].yyyy, reg_tmp0)).xyz;
return false;
}
bool sub_20() {
reg_tmp1 = uniforms.f[95].yyyy;
reg_tmp4.zw = (fma_s(reg_tmp6.xyxy, reg_tmp1, reg_tmp1)).zw;
reg_tmp4.zw = (fma_s(reg_tmp4, uniforms.f[42].wzyx, reg_tmp3)).zw;
return false;
}
bool sub_6() {
reg_tmp2.x = (uniforms.f[94].xxxx).x;
reg_tmp2.y = (uniforms.f[95].wwww).y;
reg_tmp2.z = (uniforms.f[93].wwww).z;
reg_tmp2.w = (uniforms.f[93].zzzz).w;
reg_tmp3.x = (mul_s(uniforms.f[93].yyyy, reg_tmp3.xxxx)).x;
reg_tmp3.y = (reg_tmp3.xxxx + -reg_tmp2.xxxx).y;
reg_tmp3.zw = (floor(reg_tmp3.xyxy)).zw;
reg_tmp3.xy = (reg_tmp3.xyyy + -reg_tmp3.zwww).xy;
reg_tmp3.xy = (fma_s(reg_tmp3.xyyy, reg_tmp2.zzzz, -reg_tmp2.yyyy)).xy;
reg_tmp3.xy = (abs(reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (fma_s(reg_tmp3.xyxy, -reg_tmp2.zzzz, reg_tmp2.wwww)).xy;
reg_tmp3.xy = (mul_s(reg_tmp3.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp3.xy = (mul_s(reg_tmp2.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (fma_s(reg_tmp3.xyyy, reg_tmp2.zzzz, -reg_tmp2.yyyy)).xy;
return false;
}
// reference: 76B8C3225FB1165B, 32B606A8D81B7AB9
// shader: 8B30, 948E95FCE7E8B16A
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
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(min((const_color[1].aaa) + (const_color[1].rgb), vec3(1)) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(1)) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1)) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((combiner_buffer.a) * (const_color[5].a), 0.0, 1.0));
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
// reference: A4814D94384865A3, 948E95FCE7E8B16A
// program: 32B606A8D81B7AB9, DADA7DA78218FFEA, 948E95FCE7E8B16A
// shader: 8B31, F2EFC8AD043F300B

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

bool sub_0();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp1 = vs_in_reg0.xyzz;
reg_tmp1.w = (uniforms.f[95].wwww).w;
reg_tmp8.x = dot_s(uniforms.f[5].wzyx, reg_tmp1);
reg_tmp8.y = dot_s(uniforms.f[6].wzyx, reg_tmp1);
reg_tmp8.z = dot_s(uniforms.f[7].wzyx, reg_tmp1);
reg_tmp8.w = (uniforms.f[95].wwww).w;
reg_tmp5 = vs_in_reg1.xyzz;
reg_tmp7.x = dot_3(uniforms.f[5].wzy, reg_tmp5.xyz);
reg_tmp7.y = dot_3(uniforms.f[6].wzy, reg_tmp5.xyz);
reg_tmp7.z = dot_3(uniforms.f[7].wzy, reg_tmp5.xyz);
reg_tmp7.w = (uniforms.f[95].zzzz).w;
reg_tmp9.x = dot_s(uniforms.f[0].wzyx, reg_tmp8);
reg_tmp9.y = dot_s(uniforms.f[1].wzyx, reg_tmp8);
reg_tmp9.z = dot_s(-uniforms.f[2].wzyx, reg_tmp8);
reg_tmp9.w = dot_s(uniforms.f[3].wzyx, reg_tmp8);
reg_tmp10.x = (uniforms.f[95].wwww).x;
reg_tmp10.x = (-uniforms.f[95].yyyy + reg_tmp10.xxxx).x;
reg_tmp10.y = (uniforms.f[95].wwww).y;
reg_tmp2.xy = (mul_s(-reg_tmp9.wwww, reg_tmp10.xyyy)).xy;
bool_regs.x = reg_tmp9.xxxx.x < reg_tmp2.xyyy.x;
bool_regs.y = reg_tmp9.xxxx.y > reg_tmp2.xyyy.y;
if (all(bool_regs)) {
sub_1();
}
vs_out_attr0 = reg_tmp9;
reg_tmp2.x = dot_3(uniforms.f[8].wzy, reg_tmp7.xyz);
reg_tmp2.y = dot_3(uniforms.f[9].wzy, reg_tmp7.xyz);
reg_tmp2.z = dot_3(uniforms.f[10].wzy, reg_tmp7.xyz);
reg_tmp2.w = (uniforms.f[95].zzzz).w;
reg_tmp6.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp6 = mul_s(reg_tmp2, reg_tmp6.xxxx);
bool_regs = equal(-uniforms.f[95].ww, reg_tmp6.zz);
reg_tmp4 = uniforms.f[95].wwww + reg_tmp6.zzzz;
reg_tmp4 = mul_s(uniforms.f[95].xxxx, reg_tmp4);
vs_out_attr3.w = (uniforms.f[95].zzzz).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[95].xxxx, reg_tmp6);
if (!bool_regs.x) {
sub_2();
} else {
sub_3();
}
reg_tmp11 = vs_in_reg2.xyxy;
if (uniforms.b[9]) {
sub_4();
} else {
sub_5();
}
vs_out_attr5.xy = (uniforms.f[95].wwww).xy;
vs_out_attr5 = uniforms.f[95].wwww;
reg_tmp10.x = dot_s(uniforms.f[8].wzyx, reg_tmp8);
reg_tmp10.y = dot_s(uniforms.f[9].wzyx, reg_tmp8);
reg_tmp10.z = dot_s(uniforms.f[10].wzyx, reg_tmp8);
reg_tmp10.w = dot_s(uniforms.f[11].wzyx, reg_tmp8);
reg_tmp3 = uniforms.f[12].wzyx;
reg_tmp3.y = (mul_s(uniforms.f[95].wwww, reg_tmp3.yyyy)).y;
reg_tmp4 = fma_s(reg_tmp11, uniforms.f[13].wzyx, reg_tmp3);
vs_out_attr2 = reg_tmp4;
vs_out_attr4 = -reg_tmp10;
vs_out_attr1 = reg_tmp0;
return true;
}
bool sub_1() {
reg_tmp9.x = (-reg_tmp9.wwww).x;
return false;
}
bool sub_2() {
vs_out_attr3.z = rcp_s(reg_tmp4.x);
vs_out_attr3.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_3() {
vs_out_attr3.x = (uniforms.f[95].wwww).x;
vs_out_attr3.yzw = (uniforms.f[95].zzzz).yzw;
return false;
}
bool sub_4() {
reg_tmp5 = mul_s(uniforms.f[94].wwww, vs_in_reg3);
reg_tmp1 = uniforms.f[94].zzzz + -reg_tmp5;
reg_tmp2.x = rcp_s(reg_tmp1.x);
reg_tmp2.y = rcp_s(reg_tmp1.y);
reg_tmp2.z = rcp_s(reg_tmp1.z);
reg_tmp5.xyz = (mul_s(reg_tmp5, reg_tmp2)).xyz;
reg_tmp0.xyz = (reg_tmp5.xyzz).xyz;
reg_tmp0.w = (reg_tmp5.wwww).w;
return false;
}
bool sub_5() {
reg_tmp0 = uniforms.f[95].wwww;
return false;
}
// reference: 3E9701036B258161, F2EFC8AD043F300B
// shader: 8B30, 495B70B9ABF582BB
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
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B6FDED3E4F4A5AEC, 495B70B9ABF582BB
// program: F2EFC8AD043F300B, DADA7DA78218FFEA, 495B70B9ABF582BB
// shader: 8B30, BDBAC4555AA91667
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
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
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
// reference: B6FDED3EA8E5E728, BDBAC4555AA91667
// program: F2EFC8AD043F300B, DADA7DA78218FFEA, BDBAC4555AA91667
// shader: 8DD9, 1E9B8787AEA67ABF

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

    vec4 vtx_color = vec4(vtx.attributes[4].x, vtx.attributes[4].y, vtx.attributes[4].z, vtx.attributes[4].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);
    texcoord1 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);

    texcoord0_w = vtx.attributes[5].z;
    view = vec3(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z);
    texcoord2 = vec2(vtx.attributes[1].z, vtx.attributes[1].w);

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
// reference: 2999884939626C3C, 1E9B8787AEA67ABF
// shader: 8B31, 2D885119A2D78020

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
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;

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
reg_tmp2.xy = (uniforms.f[95].wwww).xy;
reg_tmp2.zw = (uniforms.f[95].zzzz).zw;
reg_tmp1 = vs_in_reg0.xyzz;
reg_tmp1.w = (uniforms.f[95].yyyy).w;
reg_tmp4 = mul_s(vs_in_reg3, reg_tmp2);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = mul_s(uniforms.f[5 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp13 = mul_s(uniforms.f[6 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp14 = mul_s(uniforms.f[7 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp12 = fma_s(reg_tmp4.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
if (uniforms.b[11]) {
sub_1();
}
reg_tmp8.x = dot_s(reg_tmp1, reg_tmp12);
reg_tmp8.y = dot_s(reg_tmp1, reg_tmp13);
reg_tmp8.z = dot_s(reg_tmp1, reg_tmp14);
reg_tmp8.w = (uniforms.f[95].yyyy).w;
vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp8);
vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp8);
vs_out_attr0.z = dot_s(-uniforms.f[2].wzyx, reg_tmp8);
vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp8);
reg_tmp10 = vs_in_reg1.xyzz;
reg_tmp9.x = dot_3(reg_tmp10.xyz, reg_tmp12.xyz);
reg_tmp9.y = dot_3(reg_tmp10.xyz, reg_tmp13.xyz);
reg_tmp9.z = dot_3(reg_tmp10.xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_3(uniforms.f[77].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[78].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[79].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[95].xxxx).w;
reg_tmp6.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp6 = mul_s(reg_tmp2, reg_tmp6.xxxx);
reg_tmp6.w = (uniforms.f[95].yyyy).w;
if (uniforms.b[8]) {
sub_3();
} else {
sub_4();
}
reg_tmp3 = uniforms.f[81].wzyx;
reg_tmp4 = fma_s(vs_in_reg2.xyxy, uniforms.f[82].wzyx, reg_tmp3);
if (uniforms.b[0]) {
sub_7();
}
reg_tmp10 = mul_s(reg_tmp6, reg_tmp6);
reg_tmp5 = mul_s(reg_tmp6.xyzz, reg_tmp6.yzzx);
reg_tmp7 = reg_tmp6;
reg_tmp7.w = (uniforms.f[95].yyyy).w;
reg_tmp11 = reg_tmp10.xxxx + -reg_tmp10.yyyy;
reg_tmp1.x = dot_s(uniforms.f[83].wzyx, reg_tmp7);
reg_tmp1.y = dot_s(uniforms.f[84].wzyx, reg_tmp7);
reg_tmp1.z = dot_s(uniforms.f[85].wzyx, reg_tmp7);
reg_tmp2.x = dot_s(uniforms.f[86].wzyx, reg_tmp5);
reg_tmp2.y = dot_s(uniforms.f[87].wzyx, reg_tmp5);
reg_tmp2.z = dot_s(uniforms.f[88].wzyx, reg_tmp5);
reg_tmp6 = fma_s(reg_tmp11.xxxx, uniforms.f[89].wzyx, reg_tmp1);
reg_tmp10.x = dot_s(uniforms.f[77].wzyx, reg_tmp8);
reg_tmp10.y = dot_s(uniforms.f[78].wzyx, reg_tmp8);
reg_tmp10.z = dot_s(uniforms.f[79].wzyx, reg_tmp8);
reg_tmp0 = reg_tmp6 + reg_tmp2;
if (uniforms.b[1]) {
sub_8();
} else {
sub_9();
}
vs_out_attr1 = reg_tmp4;
vs_out_attr3 = -reg_tmp10;
vs_out_attr4.xyz = (mul_s(uniforms.f[94].wwww, reg_tmp0)).xyz;
vs_out_attr4.w = (mul_s(uniforms.f[94].yyyy, vs_in_reg1.wwww)).w;
return true;
}
bool sub_1() {
reg_tmp4 = mul_s(vs_in_reg5, reg_tmp2);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = fma_s(reg_tmp4.zzzz, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.zzzz, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.zzzz, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp4.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
if (uniforms.b[12]) {
sub_2();
}
return false;
}
bool sub_2() {
reg_tmp4 = mul_s(uniforms.f[95].wwww, vs_in_reg6);
reg_tmp5 = mul_s(uniforms.f[95].zzzz, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = fma_s(reg_tmp5.xxxx, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.xxxx, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.xxxx, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp5.yyyy, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.yyyy, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.yyyy, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
addr_regs.xy = ivec2(reg_tmp4.zw);
reg_tmp12 = fma_s(reg_tmp5.zzzz, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.zzzz, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.zzzz, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp5.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
return false;
}
bool sub_3() {
reg_tmp15 = reg_tmp9;
reg_tmp10 = vs_in_reg4.xyzz;
reg_tmp9.x = dot_3(reg_tmp10.xyz, reg_tmp12.xyz);
reg_tmp9.y = dot_3(reg_tmp10.xyz, reg_tmp13.xyz);
reg_tmp9.z = dot_3(reg_tmp10.xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_3(uniforms.f[77].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[78].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[79].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[95].xxxx).w;
reg_tmp9.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp9.x = rsq_s(reg_tmp9.x);
reg_tmp9 = mul_s(reg_tmp2, reg_tmp9.xxxx);
reg_tmp9.w = (uniforms.f[95].yyyy).w;
reg_tmp2 = vec4(lessThan(vs_in_reg4.wwww, -vs_in_reg4.wwww));
reg_tmp3 = vec4(lessThan(-vs_in_reg4.wwww, vs_in_reg4.wwww));
reg_tmp0 = reg_tmp3 + -reg_tmp2;
reg_tmp7 = mul_s(reg_tmp6.yzxx, reg_tmp9.zxyy);
reg_tmp7 = fma_s(-reg_tmp9.yzxx, reg_tmp6.zxyy, reg_tmp7);
reg_tmp7.w = dot_3(reg_tmp7.xyz, reg_tmp7.xyz);
reg_tmp7.w = rsq_s(reg_tmp7.w);
reg_tmp7 = mul_s(reg_tmp7, reg_tmp7.wwww);
reg_tmp7 = mul_s(reg_tmp7, reg_tmp0);
reg_tmp11.w = (reg_tmp6.zzzz + reg_tmp7.yyyy).w;
reg_tmp9 = mul_s(reg_tmp7.yzxx, reg_tmp6.zxyy);
reg_tmp9 = fma_s(-reg_tmp6.yzxx, reg_tmp7.zxyy, reg_tmp9);
reg_tmp11.w = (reg_tmp9.xxxx + reg_tmp11).w;
reg_tmp9.w = (reg_tmp7.zzzz).w;
reg_tmp7.z = (reg_tmp9.xxxx).z;
reg_tmp11.w = (uniforms.f[95].yyyy + reg_tmp11).w;
reg_tmp6.w = (reg_tmp7.xxxx).w;
reg_tmp7.x = (reg_tmp6.zzzz).x;
reg_tmp11.x = (uniforms.f[95].yyyy).x;
reg_tmp11.y = (-uniforms.f[95].yyyy).y;
reg_tmp7.xz = (reg_tmp9.wwyy + -reg_tmp6.yyww).xz;
reg_tmp7.y = (reg_tmp6.xxxx + -reg_tmp9.zzzz).y;
reg_tmp7.w = (reg_tmp11).w;
reg_tmp11 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp11 = vec4(rsq_s(reg_tmp11.x));
vs_out_attr2 = mul_s(reg_tmp7, reg_tmp11);
reg_tmp9 = reg_tmp15;
return false;
}
bool sub_4() {
bool_regs = equal(-uniforms.f[95].yy, reg_tmp6.zz);
reg_tmp4 = uniforms.f[95].yyyy + reg_tmp6.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].wwww, reg_tmp4);
vs_out_attr2.w = (uniforms.f[95].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].wwww, reg_tmp6);
if (!bool_regs.x) {
sub_5();
} else {
sub_6();
}
return false;
}
bool sub_5() {
vs_out_attr2.z = rcp_s(reg_tmp4.x);
vs_out_attr2.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_6() {
vs_out_attr2.x = (uniforms.f[95].yyyy).x;
vs_out_attr2.yzw = (uniforms.f[95].xxxx).yzw;
return false;
}
bool sub_7() {
reg_tmp1 = uniforms.f[94].wwww;
reg_tmp4.zw = (fma_s(reg_tmp6.xyxy, reg_tmp1, reg_tmp1)).zw;
reg_tmp4.zw = (fma_s(reg_tmp4, uniforms.f[82].wzyx, reg_tmp3)).zw;
return false;
}
bool sub_8() {
reg_tmp3 = -uniforms.f[90].wzyx + reg_tmp8;
reg_tmp3.w = (uniforms.f[95].xxxx).w;
reg_tmp11.x = dot_s(reg_tmp3, reg_tmp3);
reg_tmp11.x = rsq_s(reg_tmp11.x);
reg_tmp11 = mul_s(reg_tmp3, reg_tmp11.xxxx);
reg_tmp9.w = (uniforms.f[95].xxxx).w;
reg_tmp12.x = dot_s(reg_tmp9, reg_tmp9);
reg_tmp12.x = rsq_s(reg_tmp12.x);
reg_tmp12 = mul_s(reg_tmp9, reg_tmp12.xxxx);
reg_tmp2 = vec4(dot_3(reg_tmp11.xyz, reg_tmp12.xyz));
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp2);
reg_tmp2 = mul_s(reg_tmp12, reg_tmp2.xxxx);
vs_out_attr5 = reg_tmp11 + -reg_tmp2;
return false;
}
bool sub_9() {
vs_out_attr5 = reg_tmp4;
return false;
}
// reference: AFE11F574338FA42, 2D885119A2D78020
// shader: 8B30, 3CBE38F7B26AE0D3
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
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position + view);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-light_src[1].position - view) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-light_src[1].position - view) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(min((rounded_primary_color.rgb) + (primary_fragment_color.rgb), vec3(1)) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((texture(tex_cube, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp(min((last_tex_env_out.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (combiner_buffer.aaa), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1)) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((combiner_buffer.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
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
// reference: 7D457F18909D4030, 3CBE38F7B26AE0D3
// program: 2D885119A2D78020, 1E9B8787AEA67ABF, 3CBE38F7B26AE0D3
// shader: 8B31, DB1A47ADA70F78C4

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
vs_out_attr0.xy = (vs_in_reg0.xyyy).xy;
vs_out_attr0.z = (uniforms.f[95].wwww).z;
vs_out_attr0.w = (uniforms.f[95].zzzz).w;
reg_tmp0 = vs_in_reg0.zwzw;
vs_out_attr1 = reg_tmp0.xyxy;
return true;
}
// reference: DABAE56B18D2F6D1, DB1A47ADA70F78C4
// shader: 8B30, A16D42D83E64F5B0
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C56D94969C2945DE, A16D42D83E64F5B0
// program: DB1A47ADA70F78C4, 6F6B8D04DE5400F0, A16D42D83E64F5B0
// shader: 8B30, A2BB67B550C2653E
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
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(min((const_color[1].aaa) + (const_color[1].rgb), vec3(1)) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(1)) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1)) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((combiner_buffer.a) * (const_color[5].a), 0.0, 1.0));
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
// reference: 977EB8A7384865A3, A2BB67B550C2653E
// program: 32B606A8D81B7AB9, DADA7DA78218FFEA, A2BB67B550C2653E
// shader: 8B30, 2DE87652989F51F5
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C56D94966A7FE2DF, 2DE87652989F51F5
// program: DB1A47ADA70F78C4, 6F6B8D04DE5400F0, 2DE87652989F51F5
// reference: C56D94967A0A1673, A16D42D83E64F5B0
// shader: 8DD9, 48795C23CA84C4B5

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

    texcoord0 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);
    texcoord1 = vec2(vtx.attributes[1].z, vtx.attributes[1].w);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);
    texcoord2 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);

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
// reference: 740EEEAAB290EC56, 48795C23CA84C4B5
// shader: 8B31, 9C3E0A217AEE1858

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0 = vs_in_reg0.xyyy;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
vs_out_attr0 = reg_tmp0;
vs_out_attr1 = vs_in_reg0.zwzw;
vs_out_attr2 = vs_in_reg0.zwzw;
return true;
}
// reference: 12FD9501CAA693E5, 9C3E0A217AEE1858
// shader: 8B30, F91538758B76B65C
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) - (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.ggg) - (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.r);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((texcolor0.bbb) - (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4 * 2.0, alpha_output_4 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5CC2BB39B95E1577, F91538758B76B65C
// program: 9C3E0A217AEE1858, 48795C23CA84C4B5, F91538758B76B65C
// shader: 8B31, 0C81BDF26F6F0F0D

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0 = vs_in_reg0.xyyy;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
vs_out_attr0 = reg_tmp0;
reg_tmp1 = vs_in_reg0.zwzw;
reg_tmp2 = vs_in_reg0.zwzw;
reg_tmp1 = uniforms.f[5].wzyx + reg_tmp1;
vs_out_attr1 = reg_tmp1;
vs_out_attr2 = reg_tmp2;
return true;
}
// reference: 7A1A4D2DE095ECE2, 0C81BDF26F6F0F0D
// shader: 8B30, 28BE256BBD09D9FB
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
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb) + (texcolor1.rgb) * (vec3(1) - (const_color[0].rgb)), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb) + (texcolor2.rgb) * (vec3(1) - (const_color[1].rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 87ECD3BD66D4FA98, 28BE256BBD09D9FB
// program: 0C81BDF26F6F0F0D, 48795C23CA84C4B5, 28BE256BBD09D9FB
// program: 9C3E0A217AEE1858, 48795C23CA84C4B5, A16D42D83E64F5B0
// shader: 8B30, 19FC9C67AA5A8379
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
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (texcolor2.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (const_color[1].a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (texcolor0.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EAC18FC3883A1B08, 19FC9C67AA5A8379
// program: 9C3E0A217AEE1858, 48795C23CA84C4B5, 19FC9C67AA5A8379
// shader: 8B31, 88E77E9A4443CCB6

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0 = vs_in_reg0.xyyy;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
reg_tmp1.xy = (vs_in_reg0.zwzw).xy;
reg_tmp1.zw = (vs_in_reg1.xyxy).zw;
reg_tmp2 = vs_in_reg1.zwzw;
vs_out_attr0 = reg_tmp0;
vs_out_attr1 = reg_tmp1;
vs_out_attr2 = reg_tmp2;
return true;
}
// reference: 789FD29B21AAFFB6, 88E77E9A4443CCB6
// shader: 8B30, E7EA048D7339CA21
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
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rgb) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[1].rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor2.rgb) * (const_color[2].rgb) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[2].rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0F41F78A14A66221, E7EA048D7339CA21
// program: 88E77E9A4443CCB6, 48795C23CA84C4B5, E7EA048D7339CA21
// shader: 8DD9, B2B7BAAB97E6D825

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

    texcoord0 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);
    texcoord1 = vec2(vtx.attributes[1].z, vtx.attributes[1].w);

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
// reference: 4A90D8FC4D93BB15, B2B7BAAB97E6D825
// shader: 8B31, CD7C1C3EFC80BEA6

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
reg_tmp0 = vs_in_reg0.xyyy;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
reg_tmp1.xy = (vs_in_reg0.zwzw).xy;
reg_tmp1.zw = (vs_in_reg1.xyxy).zw;
vs_out_attr0 = reg_tmp0;
vs_out_attr1 = reg_tmp1;
return true;
}
// reference: 0C381F0B8F2A009A, CD7C1C3EFC80BEA6
// shader: 8B30, 7DB39454A11F349C
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F1C2B259F6EF8AEA, 7DB39454A11F349C
// program: CD7C1C3EFC80BEA6, B2B7BAAB97E6D825, 7DB39454A11F349C
// reference: B6FDED3E5D016BAD, BDBAC4555AA91667
// reference: B6FDED3EBAAED669, 495B70B9ABF582BB
// shader: 8B31, 261DF6ADD2023E38

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
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;

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
reg_tmp2.xy = (uniforms.f[95].wwww).xy;
reg_tmp2.zw = (uniforms.f[95].zzzz).zw;
reg_tmp1 = vs_in_reg0.xyzz;
reg_tmp1.w = (uniforms.f[95].yyyy).w;
reg_tmp4 = mul_s(vs_in_reg3, reg_tmp2);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = mul_s(uniforms.f[5 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp13 = mul_s(uniforms.f[6 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp14 = mul_s(uniforms.f[7 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp12 = fma_s(reg_tmp4.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
if (uniforms.b[11]) {
sub_1();
}
reg_tmp8.x = dot_s(reg_tmp1, reg_tmp12);
reg_tmp8.y = dot_s(reg_tmp1, reg_tmp13);
reg_tmp8.z = dot_s(reg_tmp1, reg_tmp14);
reg_tmp8.w = (uniforms.f[95].yyyy).w;
vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp8);
vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp8);
vs_out_attr0.z = dot_s(-uniforms.f[2].wzyx, reg_tmp8);
vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp8);
reg_tmp10 = vs_in_reg1.xyzz;
reg_tmp9.x = dot_3(reg_tmp10.xyz, reg_tmp12.xyz);
reg_tmp9.y = dot_3(reg_tmp10.xyz, reg_tmp13.xyz);
reg_tmp9.z = dot_3(reg_tmp10.xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_3(uniforms.f[77].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[78].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[79].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[95].xxxx).w;
reg_tmp6.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp6 = mul_s(reg_tmp2, reg_tmp6.xxxx);
reg_tmp6.w = (uniforms.f[95].yyyy).w;
if (uniforms.b[8]) {
sub_3();
} else {
sub_4();
}
if (uniforms.b[0]) {
sub_7();
} else {
sub_8();
}
vs_out_attr5 = reg_tmp7.xyzz;
reg_tmp3 = uniforms.f[85].wzyx;
reg_tmp4 = fma_s(vs_in_reg2.xyxy, uniforms.f[86].wzyx, reg_tmp3);
if (uniforms.b[1]) {
sub_9();
}
reg_tmp10 = mul_s(reg_tmp6, reg_tmp6);
reg_tmp5 = mul_s(reg_tmp6.xyzz, reg_tmp6.yzzx);
reg_tmp7 = reg_tmp6;
reg_tmp7.w = (uniforms.f[95].yyyy).w;
reg_tmp11 = reg_tmp10.xxxx + -reg_tmp10.yyyy;
reg_tmp1.x = dot_s(uniforms.f[87].wzyx, reg_tmp7);
reg_tmp1.y = dot_s(uniforms.f[88].wzyx, reg_tmp7);
reg_tmp1.z = dot_s(uniforms.f[89].wzyx, reg_tmp7);
reg_tmp2.x = dot_s(uniforms.f[90].wzyx, reg_tmp5);
reg_tmp2.y = dot_s(uniforms.f[91].wzyx, reg_tmp5);
reg_tmp2.z = dot_s(uniforms.f[92].wzyx, reg_tmp5);
reg_tmp6 = fma_s(reg_tmp11.xxxx, uniforms.f[93].wzyx, reg_tmp1);
reg_tmp10.x = dot_s(uniforms.f[77].wzyx, reg_tmp8);
reg_tmp10.y = dot_s(uniforms.f[78].wzyx, reg_tmp8);
reg_tmp10.z = dot_s(uniforms.f[79].wzyx, reg_tmp8);
reg_tmp0 = reg_tmp6 + reg_tmp2;
vs_out_attr1 = reg_tmp4;
vs_out_attr3 = -reg_tmp10;
vs_out_attr4.xyz = (mul_s(uniforms.f[94].wwww, reg_tmp0)).xyz;
vs_out_attr4.w = (mul_s(uniforms.f[94].zzzz, vs_in_reg1.wwww)).w;
return true;
}
bool sub_1() {
reg_tmp4 = mul_s(vs_in_reg5, reg_tmp2);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = fma_s(reg_tmp4.zzzz, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.zzzz, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.zzzz, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp4.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
if (uniforms.b[12]) {
sub_2();
}
return false;
}
bool sub_2() {
reg_tmp4 = mul_s(uniforms.f[95].wwww, vs_in_reg6);
reg_tmp5 = mul_s(uniforms.f[95].zzzz, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = fma_s(reg_tmp5.xxxx, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.xxxx, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.xxxx, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp5.yyyy, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.yyyy, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.yyyy, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
addr_regs.xy = ivec2(reg_tmp4.zw);
reg_tmp12 = fma_s(reg_tmp5.zzzz, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.zzzz, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.zzzz, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp5.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
return false;
}
bool sub_3() {
reg_tmp10 = vs_in_reg4.xyzz;
reg_tmp9.x = dot_3(reg_tmp10.xyz, reg_tmp12.xyz);
reg_tmp9.y = dot_3(reg_tmp10.xyz, reg_tmp13.xyz);
reg_tmp9.z = dot_3(reg_tmp10.xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_3(uniforms.f[77].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[78].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[79].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[95].xxxx).w;
reg_tmp9.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp9.x = rsq_s(reg_tmp9.x);
reg_tmp9 = mul_s(reg_tmp2, reg_tmp9.xxxx);
reg_tmp9.w = (uniforms.f[95].yyyy).w;
reg_tmp2 = vec4(lessThan(vs_in_reg4.wwww, -vs_in_reg4.wwww));
reg_tmp3 = vec4(lessThan(-vs_in_reg4.wwww, vs_in_reg4.wwww));
reg_tmp0 = reg_tmp3 + -reg_tmp2;
reg_tmp7 = mul_s(reg_tmp6.yzxx, reg_tmp9.zxyy);
reg_tmp7 = fma_s(-reg_tmp9.yzxx, reg_tmp6.zxyy, reg_tmp7);
reg_tmp7.w = dot_3(reg_tmp7.xyz, reg_tmp7.xyz);
reg_tmp7.w = rsq_s(reg_tmp7.w);
reg_tmp7 = mul_s(reg_tmp7, reg_tmp7.wwww);
reg_tmp7 = mul_s(reg_tmp7, reg_tmp0);
reg_tmp11.w = (reg_tmp6.zzzz + reg_tmp7.yyyy).w;
reg_tmp9 = mul_s(reg_tmp7.yzxx, reg_tmp6.zxyy);
reg_tmp9 = fma_s(-reg_tmp6.yzxx, reg_tmp7.zxyy, reg_tmp9);
reg_tmp11.w = (reg_tmp9.xxxx + reg_tmp11).w;
reg_tmp9.w = (reg_tmp7.zzzz).w;
reg_tmp7.z = (reg_tmp9.xxxx).z;
reg_tmp11.w = (uniforms.f[95].yyyy + reg_tmp11).w;
reg_tmp6.w = (reg_tmp7.xxxx).w;
reg_tmp7.x = (reg_tmp6.zzzz).x;
reg_tmp11.x = (uniforms.f[95].yyyy).x;
reg_tmp11.y = (-uniforms.f[95].yyyy).y;
reg_tmp7.xz = (reg_tmp9.wwyy + -reg_tmp6.yyww).xz;
reg_tmp7.y = (reg_tmp6.xxxx + -reg_tmp9.zzzz).y;
reg_tmp7.w = (reg_tmp11).w;
reg_tmp11 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp11 = vec4(rsq_s(reg_tmp11.x));
vs_out_attr2 = mul_s(reg_tmp7, reg_tmp11);
return false;
}
bool sub_4() {
bool_regs = equal(-uniforms.f[95].yy, reg_tmp6.zz);
reg_tmp4 = uniforms.f[95].yyyy + reg_tmp6.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].wwww, reg_tmp4);
vs_out_attr2.w = (uniforms.f[95].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].wwww, reg_tmp6);
if (!bool_regs.x) {
sub_5();
} else {
sub_6();
}
return false;
}
bool sub_5() {
vs_out_attr2.z = rcp_s(reg_tmp4.x);
vs_out_attr2.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_6() {
vs_out_attr2.x = (uniforms.f[95].yyyy).x;
vs_out_attr2.yzw = (uniforms.f[95].xxxx).yzw;
return false;
}
bool sub_7() {
reg_tmp4 = -uniforms.f[81].wzyx + reg_tmp8;
reg_tmp4.w = (uniforms.f[95].xxxx).w;
reg_tmp5.x = dot_s(reg_tmp4, reg_tmp4);
reg_tmp5.x = rsq_s(reg_tmp5.x);
reg_tmp5 = mul_s(reg_tmp4, reg_tmp5.xxxx);
reg_tmp7.x = dot_s(uniforms.f[82].wzyx, reg_tmp5);
reg_tmp7.y = dot_s(uniforms.f[83].wzyx, reg_tmp5);
reg_tmp7.z = dot_s(uniforms.f[84].wzyx, reg_tmp5);
return false;
}
bool sub_8() {
reg_tmp7.x = dot_s(uniforms.f[82].wzyx, reg_tmp8);
reg_tmp7.y = dot_s(uniforms.f[83].wzyx, reg_tmp8);
reg_tmp7.z = dot_s(uniforms.f[84].wzyx, reg_tmp8);
return false;
}
bool sub_9() {
reg_tmp1 = uniforms.f[94].wwww;
reg_tmp4.zw = (fma_s(reg_tmp6.xyxy, reg_tmp1, reg_tmp1)).zw;
reg_tmp4.zw = (fma_s(reg_tmp4, uniforms.f[86].wzyx, reg_tmp3)).zw;
return false;
}
// reference: 7566EE642A8D306B, 261DF6ADD2023E38
// shader: 8B30, D75DA5D62C87B29D
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
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(min((const_color[1].aaa) + (const_color[1].rgb), vec3(1)) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(1)) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1)) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((combiner_buffer.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
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
// reference: A0ED5D86384865A3, D75DA5D62C87B29D
// program: 261DF6ADD2023E38, 1E9B8787AEA67ABF, D75DA5D62C87B29D
// shader: 8B30, CE5E42F77BB85831
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
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(min((const_color[1].aaa) + (const_color[1].rgb), vec3(1)) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(1)) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1)) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((combiner_buffer.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
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
// reference: C24CB9EE691674F8, CE5E42F77BB85831
// program: 261DF6ADD2023E38, 1E9B8787AEA67ABF, CE5E42F77BB85831
// reference: A0ED5D86CDACE926, D75DA5D62C87B29D
// shader: 8B30, 0444B5A69EEA5051
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
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
// reference: ACAF45A35D016BAD, 0444B5A69EEA5051
// program: F2EFC8AD043F300B, DADA7DA78218FFEA, 0444B5A69EEA5051
// reference: 9F50B090A8E5E728, 0444B5A69EEA5051
// program: 32B606A8D81B7AB9, DADA7DA78218FFEA, D75DA5D62C87B29D
// reference: C24CB9EE9CF2F87D, CE5E42F77BB85831
// shader: 8B30, 395E673E979F3112
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
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(min((const_color[1].aaa) + (const_color[1].rgb), vec3(1)) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(1)) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1)) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((combiner_buffer.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
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
// reference: 9312A8B5CDACE926, 395E673E979F3112
// program: 261DF6ADD2023E38, 1E9B8787AEA67ABF, 395E673E979F3112
// program: 32B606A8D81B7AB9, DADA7DA78218FFEA, 395E673E979F3112
// reference: 9F50B0905D016BAD, 0444B5A69EEA5051
// shader: 8B30, 2DBD72A9C6FA5C3D
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
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position + view);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-light_src[1].position - view) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-light_src[1].position - view) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(min((const_color[1].aaa) + (const_color[1].rgb), vec3(1)) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(1)) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1)) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((combiner_buffer.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
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
// reference: 23A0F1D6E8F402E8, 2DBD72A9C6FA5C3D
// program: 261DF6ADD2023E38, 1E9B8787AEA67ABF, 2DBD72A9C6FA5C3D
// reference: 977EB8A7CDACE926, A2BB67B550C2653E
// shader: 8DD9, BD6EFF0A593527B3

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)
layout(points) in;
layout(triangle_strip, max_vertices=30) out;
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
layout(location=7) in vec4 vs_out_attr7[];

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms gs_uniforms
layout(binding=3, std140) uniform gs_config {
    pica_uniforms uniforms;
};
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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);
    texcoord1 = vec2(0.0, 0.0);

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

Vertex output_buffer;
Vertex prim_buffer[3];
uint vertex_id = 0u;
bool prim_emit = false;
bool winding = false;
void setemit(uint vertex_id_, bool prim_emit_, bool winding_);
void emit();
void main() {
    output_buffer.attributes[0] = vec4(0.0, 0.0, 0.0, 1.0);
    output_buffer.attributes[1] = vec4(0.0, 0.0, 0.0, 1.0);
    output_buffer.attributes[2] = vec4(0.0, 0.0, 0.0, 1.0);
    exec_shader();
}

void setemit(uint vertex_id_, bool prim_emit_, bool winding_) {
    vertex_id = vertex_id_;
    prim_emit = prim_emit_;
    winding = winding_;
}
void emit() {
    prim_buffer[vertex_id] = output_buffer;
    if (prim_emit) {
        if (winding) {
            EmitPrim(prim_buffer[1], prim_buffer[0], prim_buffer[2]);
            winding = false;
        } else {
            EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
        }
    }
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
bool sub_9();
bool sub_10();
bool sub_6();
bool sub_7();
bool sub_8();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
uint jmp_to = 88u;
while (true) {
switch (jmp_to) {
case 88u:
bool_regs = lessThan(uniforms.f[82].zz, vs_out_attr3[0].ww);
if (bool_regs.x) {
jmp_to = 298u; break;
}
reg_tmp8.x = (vs_out_attr3[0].wwww).x;
reg_tmp9.x = (mul_s(uniforms.f[80].zzzz, reg_tmp8.xxxx)).x;
reg_tmp9.x = (floor(reg_tmp9.xxxx)).x;
reg_tmp9.x = (mul_s(uniforms.f[82].yyyy, reg_tmp9.xxxx)).x;
reg_tmp8.x = (reg_tmp8.xxxx + -reg_tmp9.xxxx).x;
reg_tmp9.y = (mul_s(uniforms.f[81].yyyy, reg_tmp8.xxxx)).y;
reg_tmp9.y = (floor(reg_tmp9.yyyy)).y;
reg_tmp9.y = (mul_s(uniforms.f[81].wwww, reg_tmp9.yyyy)).y;
reg_tmp8.x = (reg_tmp8.xxxx + -reg_tmp9.yyyy).x;
reg_tmp7.x = (reg_tmp8.xxxx).x;
reg_tmp7.y = (uniforms.f[81].zzzz).y;
reg_tmp2.xy = (vs_out_attr2[0].xyyy).xy;
reg_tmp3.xy = (vs_out_attr2[0].zyyy).xy;
reg_tmp4.xy = (vs_out_attr2[0].xwww).xy;
reg_tmp5.xy = (vs_out_attr2[0].zwww).xy;
bool_regs = lessThan(uniforms.f[81].xx, reg_tmp7.xy);
if (bool_regs.x) {
sub_1();
}
bool_regs = lessThan(uniforms.f[81].xx, reg_tmp9.xy);
if (bool_regs.y) {
sub_2();
}
if (bool_regs.x) {
sub_3();
}
reg_tmp7.xy = (reg_tmp2.xyyy).xy;
reg_tmp7.z = (reg_tmp3.xxxx).z;
reg_tmp7.w = (reg_tmp3.yyyy).w;
reg_tmp8.xy = (reg_tmp4.xyyy).xy;
reg_tmp8.z = (reg_tmp5.xxxx).z;
reg_tmp8.w = (reg_tmp5.yyyy).w;
reg_tmp6.xy = (reg_tmp15.zwww).xy;
bool_regs = lessThan(uniforms.f[81].xx, reg_tmp6.xy);
if (bool_regs.y) {
jmp_to = 224u; break;
}
reg_tmp1.xy = (vs_out_attr3[0].xyyy).xy;
reg_tmp1.z = (mul_s(uniforms.f[81].yyyy, vs_out_attr3[0].xxxx)).z;
reg_tmp1.w = (mul_s(uniforms.f[81].yyyy, vs_out_attr3[0].yyyy)).w;
reg_tmp0.x = (vs_out_attr5[0].zzzz).x;
reg_tmp0.y = (vs_out_attr6[0].zzzz).y;
reg_tmp0.z = (vs_out_attr7[0].zzzz).z;
reg_tmp9.x = (mul_s(reg_tmp0.xxxx, vs_out_attr0[0].wwww)).x;
reg_tmp9.y = (mul_s(reg_tmp0.yyyy, vs_out_attr0[0].wwww)).y;
reg_tmp9.z = (mul_s(reg_tmp0.zzzz, vs_out_attr0[0].wwww)).z;
reg_tmp2.x = (vs_out_attr4[0].xxxx).x;
reg_tmp2.y = (vs_out_attr3[0].zzzz).y;
reg_tmp2.y = (reg_tmp2.yyyy + vs_out_attr4[0].zzzz).y;
reg_tmp3.x = rcp_s(uniforms.f[80].x);
reg_tmp3.x = (mul_s(reg_tmp2.yyyy, reg_tmp3.xxxx)).x;
reg_tmp3.x = (floor(reg_tmp3.xxxx)).x;
reg_tmp3.x = (mul_s(uniforms.f[80].xxxx, reg_tmp3.xxxx)).x;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp3.xxxx).y;
reg_tmp2.y = (-uniforms.f[80].yyyy + reg_tmp2.yyyy).y;
reg_tmp3 = uniforms.f[95];
reg_tmp4 = uniforms.f[94];
reg_tmp0.z = (mul_s(reg_tmp2.yyyy, reg_tmp2.yyyy)).z;
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp3.xyyy, reg_tmp3.zwww)).xy;
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp0.xyyy, reg_tmp4.xyyy)).xy;
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp0.xyyy, reg_tmp4.zwww)).xy;
reg_tmp3 = uniforms.f[93];
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp0.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp0.xyyy, reg_tmp3.zwww)).xy;
reg_tmp0.y = (mul_s(reg_tmp0.yyyy, reg_tmp2.yyyy)).y;
reg_tmp0.xy = (mul_s(-uniforms.f[81].zzzz, reg_tmp0.xyyy)).xy;
bool_regs = lessThan(uniforms.f[81].xx, reg_tmp10.zz);
if (bool_regs.x) {
sub_4();
} else {
sub_5();
}
reg_tmp10 = reg_tmp0.xyxy;
reg_tmp2 = -reg_tmp3.xwxw;
reg_tmp4 = mul_s(reg_tmp2, reg_tmp10.xxyy);
reg_tmp2.x = (reg_tmp4.zzzz + -reg_tmp4.yyyy).x;
reg_tmp2.y = (reg_tmp4.xxxx + reg_tmp4.wwww).y;
reg_tmp2.zw = (uniforms.f[81].xxxx).zw;
sub_6();
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (reg_tmp7.xyyy).xy;
setemit(0u, false, false);
emit();
reg_tmp2.xz = (reg_tmp3.zzzz).xz;
reg_tmp2.yw = (-reg_tmp3.wwww).yw;
reg_tmp4 = mul_s(reg_tmp2, reg_tmp10.xxyy);
reg_tmp2.x = (reg_tmp4.zzzz + -reg_tmp4.yyyy).x;
reg_tmp2.y = (reg_tmp4.xxxx + reg_tmp4.wwww).y;
reg_tmp2.zw = (uniforms.f[81].xxxx).zw;
sub_6();
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (reg_tmp7.zwww).xy;
setemit(1u, false, false);
emit();
reg_tmp2.xz = (-reg_tmp3.xxxx).xz;
reg_tmp2.yw = (reg_tmp3.yyyy).yw;
reg_tmp4 = mul_s(reg_tmp2, reg_tmp10.xxyy);
reg_tmp2.x = (reg_tmp4.zzzz + -reg_tmp4.yyyy).x;
reg_tmp2.y = (reg_tmp4.xxxx + reg_tmp4.wwww).y;
reg_tmp2.zw = (uniforms.f[81].xxxx).zw;
sub_6();
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (reg_tmp8.xyyy).xy;
setemit(2u, true, false);
emit();
reg_tmp2 = reg_tmp3.zyzy;
reg_tmp4 = mul_s(reg_tmp2, reg_tmp10.xxyy);
reg_tmp2.x = (reg_tmp4.zzzz + -reg_tmp4.yyyy).x;
reg_tmp2.y = (reg_tmp4.xxxx + reg_tmp4.wwww).y;
reg_tmp2.zw = (uniforms.f[81].xxxx).zw;
sub_6();
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (reg_tmp8.zwww).xy;
setemit(0u, true, true);
emit();
return true;
case 224u:
reg_tmp1.xy = (-reg_tmp10.xyyy).xy;
reg_tmp1.z = (reg_tmp15.xxxx + -reg_tmp10.xxxx).z;
reg_tmp1.w = (reg_tmp15.yyyy + -reg_tmp10.yyyy).w;
reg_tmp9.x = (reg_tmp10.wwww).x;
reg_tmp2.y = (reg_tmp15.zzzz).y;
reg_tmp2.y = (reg_tmp2.yyyy + vs_out_attr4[0].zzzz).y;
reg_tmp3.x = rcp_s(uniforms.f[80].x);
reg_tmp3.x = (mul_s(reg_tmp2.yyyy, reg_tmp3.xxxx)).x;
reg_tmp3.x = (floor(reg_tmp3.xxxx)).x;
reg_tmp3.x = (mul_s(uniforms.f[80].xxxx, reg_tmp3.xxxx)).x;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp3.xxxx).y;
reg_tmp2.y = (-uniforms.f[80].yyyy + reg_tmp2.yyyy).y;
reg_tmp3 = uniforms.f[95];
reg_tmp4 = uniforms.f[94];
reg_tmp0.z = (mul_s(reg_tmp2.yyyy, reg_tmp2.yyyy)).z;
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp3.xyyy, reg_tmp3.zwww)).xy;
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp0.xyyy, reg_tmp4.xyyy)).xy;
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp0.xyyy, reg_tmp4.zwww)).xy;
reg_tmp3 = uniforms.f[93];
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp0.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp0.xy = (fma_s(reg_tmp0.zzzz, reg_tmp0.xyyy, reg_tmp3.zwww)).xy;
reg_tmp0.y = (mul_s(reg_tmp0.yyyy, reg_tmp2.yyyy)).y;
reg_tmp0.xy = (mul_s(-uniforms.f[81].zzzz, reg_tmp0.xyyy)).xy;
reg_tmp10 = reg_tmp0.xyxy;
reg_tmp0 = mul_s(reg_tmp1.xwxw, reg_tmp10.xxyy);
reg_tmp6.x = (reg_tmp0.zzzz + -reg_tmp0.yyyy).x;
reg_tmp6.y = (reg_tmp0.xxxx + reg_tmp0.wwww).y;
reg_tmp6 = mul_s(reg_tmp6, reg_tmp9.xxxx);
reg_tmp3.xyz = (vs_out_attr0[0].xyzz).xyz;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp6.xyyy).xy;
reg_tmp3.w = (uniforms.f[81].zzzz).w;
sub_6();
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (reg_tmp7.xyyy).xy;
setemit(0u, false, false);
emit();
reg_tmp0 = mul_s(reg_tmp1.zwzw, reg_tmp10.xxyy);
reg_tmp6.x = (reg_tmp0.zzzz + -reg_tmp0.yyyy).x;
reg_tmp6.y = (reg_tmp0.xxxx + reg_tmp0.wwww).y;
reg_tmp6 = mul_s(reg_tmp6, reg_tmp9.xxxx);
reg_tmp3.xyz = (vs_out_attr0[0].xyzz).xyz;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp6.xyyy).xy;
reg_tmp3.w = (uniforms.f[81].zzzz).w;
sub_6();
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (reg_tmp7.zwww).xy;
setemit(1u, false, false);
emit();
reg_tmp0 = mul_s(reg_tmp1.xyxy, reg_tmp10.xxyy);
reg_tmp6.x = (reg_tmp0.zzzz + -reg_tmp0.yyyy).x;
reg_tmp6.y = (reg_tmp0.xxxx + reg_tmp0.wwww).y;
reg_tmp6 = mul_s(reg_tmp6, reg_tmp9.xxxx);
reg_tmp3.xyz = (vs_out_attr0[0].xyzz).xyz;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp6.xyyy).xy;
reg_tmp3.w = (uniforms.f[81].zzzz).w;
sub_6();
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (reg_tmp8.xyyy).xy;
setemit(2u, true, false);
emit();
reg_tmp0 = mul_s(reg_tmp1.zyzy, reg_tmp10.xxyy);
reg_tmp6.x = (reg_tmp0.zzzz + -reg_tmp0.yyyy).x;
reg_tmp6.y = (reg_tmp0.xxxx + reg_tmp0.wwww).y;
reg_tmp6 = mul_s(reg_tmp6, reg_tmp9.xxxx);
reg_tmp3.xyz = (vs_out_attr0[0].xyzz).xyz;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp6.xyyy).xy;
reg_tmp3.w = (uniforms.f[81].zzzz).w;
sub_6();
output_buffer.attributes[1] = vs_out_attr1[0];
output_buffer.attributes[2].xy = (reg_tmp8.zwww).xy;
setemit(0u, true, true);
emit();
return true;
case 298u:
reg_tmp10 = vs_out_attr0[0].xyzz;
reg_tmp11 = vs_out_attr4[0];
reg_tmp12 = vs_out_attr5[0];
reg_tmp13 = vs_out_attr6[0];
reg_tmp14 = vs_out_attr7[0];
reg_tmp0.xy = (vs_out_attr3[0].zzzz).xy;
bool_regs = lessThan(uniforms.f[81].xx, reg_tmp0.xy);
if (bool_regs.x) {
sub_9();
} else {
sub_10();
}
return true;
default: return false;
}
}
return false;
}
bool sub_1() {
reg_tmp0.xy = (reg_tmp2.xyyy).xy;
reg_tmp2.xy = (reg_tmp3.xyyy).xy;
reg_tmp3.xy = (reg_tmp0.xyyy).xy;
reg_tmp0.xy = (reg_tmp4.xyyy).xy;
reg_tmp4.xy = (reg_tmp5.xyyy).xy;
reg_tmp5.xy = (reg_tmp0.xyyy).xy;
return false;
}
bool sub_2() {
reg_tmp0.xy = (reg_tmp2.xyyy).xy;
reg_tmp2.xy = (reg_tmp4.xyyy).xy;
reg_tmp4.xy = (reg_tmp0.xyyy).xy;
reg_tmp0.xy = (reg_tmp3.xyyy).xy;
reg_tmp3.xy = (reg_tmp5.xyyy).xy;
reg_tmp5.xy = (reg_tmp0.xyyy).xy;
return false;
}
bool sub_3() {
reg_tmp0.xy = (reg_tmp2.xyyy).xy;
reg_tmp2.xy = (reg_tmp4.xyyy).xy;
reg_tmp4.xy = (reg_tmp5.xyyy).xy;
reg_tmp5.xy = (reg_tmp3.xyyy).xy;
reg_tmp3.xy = (reg_tmp0.xyyy).xy;
return false;
}
bool sub_4() {
reg_tmp3.xy = (reg_tmp10.xyyy).xy;
reg_tmp3.z = (reg_tmp1.xxxx + -reg_tmp10.xxxx).z;
reg_tmp3.w = (reg_tmp1.yyyy + -reg_tmp10.yyyy).w;
reg_tmp3 = mul_s(reg_tmp3, reg_tmp2.xxxx);
reg_tmp3.x = (mul_s(reg_tmp3.xxxx, vs_out_attr4[0].yyyy)).x;
reg_tmp3.z = (mul_s(reg_tmp3.zzzz, vs_out_attr4[0].yyyy)).z;
return false;
}
bool sub_5() {
reg_tmp3.x = (reg_tmp1.zzzz).x;
reg_tmp3.y = (reg_tmp1.wwww).y;
reg_tmp3.z = (reg_tmp1.zzzz).z;
reg_tmp3.w = (reg_tmp1.wwww).w;
reg_tmp3 = mul_s(reg_tmp3, reg_tmp2.xxxx);
reg_tmp3.xz = (mul_s(reg_tmp3.xzzz, vs_out_attr4[0].yyyy)).xz;
return false;
}
bool sub_9() {
reg_tmp10.w = (vs_out_attr0[0].wwww).w;
reg_tmp15.xy = (vs_out_attr3[0].xyyy).xy;
reg_tmp15.z = (vs_out_attr0[0].zzzz).z;
reg_tmp15.w = (uniforms.f[81].zzzz).w;
return false;
}
bool sub_10() {
reg_tmp15 = uniforms.f[81].xxxx;
return false;
}
bool sub_6() {
reg_tmp6.xy = (reg_tmp15.zwww).xy;
bool_regs = lessThan(uniforms.f[81].xx, reg_tmp6.xy);
if (bool_regs.y) {
sub_7();
} else {
sub_8();
}
return false;
}
bool sub_7() {
output_buffer.attributes[0].x = dot_s(reg_tmp3, reg_tmp11);
output_buffer.attributes[0].y = dot_s(reg_tmp3, reg_tmp12);
output_buffer.attributes[0].z = dot_s(reg_tmp3, reg_tmp13);
output_buffer.attributes[0].w = dot_s(reg_tmp3, reg_tmp14);
return false;
}
bool sub_8() {
reg_tmp6.x = dot_s(reg_tmp2, vs_out_attr5[0]);
reg_tmp6.y = dot_s(reg_tmp2, vs_out_attr6[0]);
reg_tmp6.z = dot_s(reg_tmp2, vs_out_attr7[0]);
reg_tmp6.xyz = (reg_tmp6.xyzz + vs_out_attr0[0].xyzz).xyz;
reg_tmp6.w = (uniforms.f[81].zzzz).w;
output_buffer.attributes[0].x = dot_s(reg_tmp6, reg_tmp11);
output_buffer.attributes[0].y = dot_s(reg_tmp6, reg_tmp12);
reg_tmp4.w = dot_s(reg_tmp6, reg_tmp14);
reg_tmp0.xyz = (reg_tmp9.xyzz + reg_tmp6.xyzz).xyz;
reg_tmp0.w = (uniforms.f[81].zzzz).w;
reg_tmp6.w = dot_s(reg_tmp0, reg_tmp14);
reg_tmp6.z = dot_s(reg_tmp0, -reg_tmp13);
reg_tmp6.z = (mul_s(reg_tmp4.wwww, reg_tmp6.zzzz)).z;
reg_tmp2 = vec4(rcp_s(reg_tmp6.w));
output_buffer.attributes[0].z = (mul_s(reg_tmp6.zzzz, reg_tmp2.zzzz)).z;
output_buffer.attributes[0].w = (reg_tmp4.wwww).w;
return false;
}
// reference: 80EADBB121F5541D, BD6EFF0A593527B3
// shader: 8B31, 22B227623BC2B584

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
layout(location=6) out vec4 vs_out_attr6;
layout(location=7) out vec4 vs_out_attr7;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
vs_out_attr7 = vec4(0, 0, 0, 1);
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

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
uint jmp_to = 0u;
while (true) {
switch (jmp_to) {
case 0u:
reg_tmp15 = uniforms.f[5].wzyx;
bool_regs = greaterThan(uniforms.f[6].wz, reg_tmp15.xy);
if (bool_regs.y) {
jmp_to = 30u; break;
}
reg_tmp1.xy = (vs_in_reg4.wwww).xy;
bool_regs = lessThan(uniforms.f[89].ww, reg_tmp1.xy);
if (bool_regs.x) {
jmp_to = 68u; break;
}
vs_out_attr0.xyz = (vs_in_reg0.xyzz).xyz;
vs_out_attr0.w = (mul_s(uniforms.f[7].wwww, vs_in_reg2.zzzz)).w;
vs_out_attr1 = mul_s(uniforms.f[90].yyyy, vs_in_reg1);
reg_tmp0.z = (vs_in_reg3.xxxx).z;
reg_tmp0.w = (vs_in_reg3.yyyy).w;
reg_tmp1.x = (uniforms.f[8].xxxx + -vs_in_reg2.yyyy).x;
reg_tmp0.y = (reg_tmp1.xxxx + -reg_tmp0.wwww).y;
reg_tmp0.x = (vs_in_reg2.xxxx).x;
reg_tmp0.xy = (mul_s(uniforms.f[8].wzzz, reg_tmp0.xyyy)).xy;
vs_out_attr2.xy = (reg_tmp0.xyyy).xy;
reg_tmp1.xy = (-uniforms.f[90].xxxx + reg_tmp0.zwww).xy;
reg_tmp1.xy = (mul_s(uniforms.f[8].wzzz, reg_tmp1.xyyy)).xy;
vs_out_attr2.z = (reg_tmp0.xxxx + reg_tmp1.xxxx).z;
vs_out_attr2.w = (reg_tmp0.yyyy + reg_tmp1.yyyy).w;
vs_out_attr3.xy = (uniforms.f[90].wwww).xy;
vs_out_attr3.z = (vs_in_reg4.zzzz).z;
vs_out_attr3.w = (vs_in_reg4.wwww).w;
vs_out_attr4.z = (uniforms.f[9].wwww).z;
vs_out_attr4.xyw = (uniforms.f[90].wwww).xyw;
vs_out_attr5 = uniforms.f[90].wwww;
vs_out_attr6 = uniforms.f[90].wwww;
vs_out_attr7 = uniforms.f[90].wwww;
return true;
case 30u:
vs_out_attr0.xyz = (vs_in_reg0.xyzz).xyz;
vs_out_attr0.w = (mul_s(uniforms.f[7].wwww, vs_in_reg2.zzzz)).w;
vs_out_attr1 = mul_s(uniforms.f[90].yyyy, vs_in_reg1);
reg_tmp0.z = (mul_s(uniforms.f[89].zzzz, vs_in_reg4.xxxx)).z;
reg_tmp0.w = (mul_s(uniforms.f[89].zzzz, vs_in_reg4.yyyy)).w;
reg_tmp1.xy = (vs_in_reg4.wwww).xy;
bool_regs = lessThan(uniforms.f[89].zz, reg_tmp1.xy);
if (bool_regs.x) {
jmp_to = 59u; break;
}
reg_tmp1.x = (uniforms.f[8].xxxx + -vs_in_reg2.yyyy).x;
reg_tmp0.y = (reg_tmp1.xxxx + -reg_tmp0.wwww).y;
reg_tmp0.x = (vs_in_reg2.xxxx).x;
reg_tmp0.xy = (mul_s(uniforms.f[8].wzzz, reg_tmp0.xyyy)).xy;
vs_out_attr2.xy = (reg_tmp0.xyyy).xy;
reg_tmp1.xy = (-uniforms.f[90].xxxx + reg_tmp0.zwww).xy;
reg_tmp1.xy = (mul_s(uniforms.f[8].wzzz, reg_tmp1.xyyy)).xy;
vs_out_attr2.z = (reg_tmp0.xxxx + reg_tmp1.xxxx).z;
vs_out_attr2.w = (reg_tmp0.yyyy + reg_tmp1.yyyy).w;
vs_out_attr3.xy = (reg_tmp0.zwww).xy;
vs_out_attr3.z = (mul_s(uniforms.f[90].zzzz, vs_in_reg3.yyyy)).z;
vs_out_attr3.w = (vs_in_reg4.wwww).w;
vs_out_attr4.x = (mul_s(uniforms.f[89].wwww, vs_in_reg3.xxxx)).x;
vs_out_attr4.y = (mul_s(uniforms.f[91].yyyy, vs_in_reg4.zzzz)).y;
vs_out_attr4.z = (uniforms.f[9].wwww).z;
vs_out_attr4.w = (uniforms.f[90].wwww).w;
vs_out_attr5 = uniforms.f[10].wzyx;
vs_out_attr6 = uniforms.f[11].wzyx;
vs_out_attr7 = uniforms.f[12].wzyx;
return true;
case 59u:
vs_out_attr2 = uniforms.f[90].wwww;
vs_out_attr3.xyz = (uniforms.f[90].wwww).xyz;
vs_out_attr3.w = (vs_in_reg4.wwww).w;
vs_out_attr4 = uniforms.f[0].wzyx;
vs_out_attr5 = uniforms.f[1].wzyx;
vs_out_attr6 = uniforms.f[2].wzyx;
vs_out_attr7 = uniforms.f[3].wzyx;
return true;
case 68u:
vs_out_attr0.xyz = (vs_in_reg0.xyzz).xyz;
vs_out_attr0.w = (mul_s(uniforms.f[89].wwww, vs_in_reg2.wwww)).w;
vs_out_attr1 = uniforms.f[90].wwww;
vs_out_attr2 = uniforms.f[90].wwww;
vs_out_attr3.x = (-uniforms.f[90].xxxx + vs_in_reg3.xxxx).x;
vs_out_attr3.y = (-uniforms.f[90].xxxx + vs_in_reg3.yyyy).y;
vs_out_attr3.z = (uniforms.f[90].xxxx).z;
vs_out_attr3.w = (vs_in_reg4.wwww).w;
bool_regs = equal(uniforms.f[90].ww, vs_in_reg4.zz);
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
return true;
default: return false;
}
}
return false;
}
bool sub_1() {
vs_out_attr4 = uniforms.f[14].wzyx;
vs_out_attr5 = uniforms.f[15].wzyx;
vs_out_attr6 = uniforms.f[16].wzyx;
vs_out_attr7 = uniforms.f[17].wzyx;
return false;
}
bool sub_2() {
vs_out_attr4 = uniforms.f[18].wzyx;
vs_out_attr5 = uniforms.f[19].wzyx;
vs_out_attr6 = uniforms.f[20].wzyx;
vs_out_attr7 = uniforms.f[21].wzyx;
return false;
}
// reference: FEF4E099E67797C7, 22B227623BC2B584
// program: 22B227623BC2B584, BD6EFF0A593527B3, 4C9684C02DD9FF08
// shader: 8B30, 96B45B900F118596
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F2FE71B70ADAFCB5, 96B45B900F118596
// program: 0D2DBB6B15798A3B, DC0A6D35EDF1ED6F, 96B45B900F118596
// shader: 8B31, 302F678AAF6F044B

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
reg_tmp1.xy = (vs_in_reg0.xyyy).xy;
reg_tmp1.zw = (uniforms.f[90].wzwz).zw;
reg_tmp0 = vs_in_reg0.zwzw;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
reg_tmp2.x = dot_s(uniforms.f[5].wzyx, reg_tmp0);
reg_tmp2.y = dot_s(uniforms.f[6].wzyx, reg_tmp0);
vs_out_attr0 = reg_tmp1;
vs_out_attr1 = reg_tmp2.xyxy;
return true;
}
// reference: 80D0425FBCFCBF76, 302F678AAF6F044B
// program: 302F678AAF6F044B, 6F6B8D04DE5400F0, 2DE87652989F51F5
// shader: 8DD9, 8F66AB8FEF098E16

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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);
    texcoord1 = vec2(0.0, 0.0);

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
// reference: 7B07DA3E334A19B0, 8F66AB8FEF098E16
// shader: 8B31, C65E9AE0C520A729

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

bool sub_0();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0 = vs_in_reg0.xyyy;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
reg_tmp1.x = dot_s(uniforms.f[5].wzyx, reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[6].wzyx, reg_tmp0);
reg_tmp2.xy = (mul_s(uniforms.f[9].wzzz, reg_tmp1.xyyy)).xy;
reg_tmp0.xy = (uniforms.f[9].yxxx + reg_tmp2.xyyy).xy;
if (uniforms.b[0]) {
sub_1();
} else {
sub_6();
}
reg_tmp1.z = (uniforms.f[10].yyyy).z;
reg_tmp0.z = (min_s(-uniforms.f[95].wwww, reg_tmp1.zzzz)).z;
vs_out_attr0.z = (max_s(-uniforms.f[95].zzzz, reg_tmp0.zzzz)).z;
vs_out_attr0.w = (uniforms.f[95].zzzz).w;
vs_out_attr1 = mul_s(uniforms.f[95].yyyy, vs_in_reg1);
reg_tmp0 = mul_s(uniforms.f[11].wzyx, vs_in_reg2.xyyy);
reg_tmp0.y = (uniforms.f[95].zzzz + -reg_tmp0.yyyy).y;
vs_out_attr2 = reg_tmp0.xyyy;
return true;
}
bool sub_1() {
reg_tmp4 = uniforms.f[90].wzyx;
bool_regs = equal(uniforms.f[4].wz, reg_tmp4.xy);
if (bool_regs.x) {
sub_2();
} else {
sub_3();
}
return false;
}
bool sub_2() {
vs_out_attr0.xy = (reg_tmp0.xyyy).xy;
return false;
}
bool sub_3() {
if (!bool_regs.y) {
sub_4();
} else {
sub_5();
}
vs_out_attr0.x = (reg_tmp0.xxxx).x;
return false;
}
bool sub_4() {
vs_out_attr0.y = (-uniforms.f[10].zzzz + reg_tmp0.yyyy).y;
return false;
}
bool sub_5() {
vs_out_attr0.y = (uniforms.f[10].zzzz + reg_tmp0.yyyy).y;
return false;
}
bool sub_6() {
vs_out_attr0.xy = (reg_tmp0.xyyy).xy;
return false;
}
// reference: 8F0B640997564EE9, C65E9AE0C520A729
// reference: C56D9496ECF9AF18, 887808CD0BBA6904
// program: C65E9AE0C520A729, 8F66AB8FEF098E16, 887808CD0BBA6904
// shader: 8B31, 10CD3F452451A509

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

bool sub_0();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp1 = vs_in_reg0.xyzz;
reg_tmp1.w = (uniforms.f[95].wwww).w;
reg_tmp8.x = dot_s(uniforms.f[5].wzyx, reg_tmp1);
reg_tmp8.y = dot_s(uniforms.f[6].wzyx, reg_tmp1);
reg_tmp8.z = dot_s(uniforms.f[7].wzyx, reg_tmp1);
reg_tmp8.w = (uniforms.f[95].wwww).w;
reg_tmp9.x = dot_s(uniforms.f[0].wzyx, reg_tmp8);
reg_tmp9.y = dot_s(uniforms.f[1].wzyx, reg_tmp8);
reg_tmp9.z = dot_s(-uniforms.f[2].wzyx, reg_tmp8);
reg_tmp9.w = dot_s(uniforms.f[3].wzyx, reg_tmp8);
reg_tmp10.x = (uniforms.f[95].wwww).x;
reg_tmp10.x = (-uniforms.f[95].zzzz + reg_tmp10.xxxx).x;
reg_tmp10.y = (uniforms.f[95].wwww).y;
reg_tmp2.xy = (mul_s(-reg_tmp9.wwww, reg_tmp10.xyyy)).xy;
bool_regs.x = reg_tmp9.xxxx.x < reg_tmp2.xyyy.x;
bool_regs.y = reg_tmp9.xxxx.y > reg_tmp2.xyyy.y;
if (all(bool_regs)) {
sub_1();
}
vs_out_attr0 = reg_tmp9;
reg_tmp10 = vs_in_reg1.xyzz;
reg_tmp9.x = dot_3(uniforms.f[5].wzy, reg_tmp10.xyz);
reg_tmp9.y = dot_3(uniforms.f[6].wzy, reg_tmp10.xyz);
reg_tmp9.z = dot_3(uniforms.f[7].wzy, reg_tmp10.xyz);
reg_tmp2.x = dot_3(uniforms.f[8].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[9].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[10].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[95].yyyy).w;
reg_tmp6.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp6 = mul_s(reg_tmp2, reg_tmp6.xxxx);
reg_tmp6.w = (uniforms.f[95].wwww).w;
reg_tmp10 = vs_in_reg4.xyzz;
reg_tmp9.x = dot_3(uniforms.f[5].wzy, reg_tmp10.xyz);
reg_tmp9.y = dot_3(uniforms.f[6].wzy, reg_tmp10.xyz);
reg_tmp9.z = dot_3(uniforms.f[7].wzy, reg_tmp10.xyz);
reg_tmp2.x = dot_3(uniforms.f[8].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[9].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[10].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[95].yyyy).w;
reg_tmp9.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp9.x = rsq_s(reg_tmp9.x);
reg_tmp9 = mul_s(reg_tmp2, reg_tmp9.xxxx);
reg_tmp9.w = (uniforms.f[95].wwww).w;
reg_tmp7 = mul_s(reg_tmp6.yzxx, reg_tmp9.zxyy);
reg_tmp7 = fma_s(-reg_tmp9.yzxx, reg_tmp6.zxyy, reg_tmp7);
reg_tmp7.w = dot_3(reg_tmp7.xyz, reg_tmp7.xyz);
reg_tmp7.w = rsq_s(reg_tmp7.w);
reg_tmp7 = mul_s(reg_tmp7, reg_tmp7.wwww);
reg_tmp11.w = (reg_tmp6.zzzz + reg_tmp7.yyyy).w;
reg_tmp9 = mul_s(reg_tmp7.yzxx, reg_tmp6.zxyy);
reg_tmp9 = fma_s(-reg_tmp6.yzxx, reg_tmp7.zxyy, reg_tmp9);
reg_tmp11.w = (reg_tmp9.xxxx + reg_tmp11).w;
reg_tmp9.w = (reg_tmp7.zzzz).w;
reg_tmp7.z = (reg_tmp9.xxxx).z;
reg_tmp11.w = (uniforms.f[95].wwww + reg_tmp11).w;
reg_tmp6.w = (reg_tmp7.xxxx).w;
reg_tmp7.x = (reg_tmp6.zzzz).x;
reg_tmp11.x = (uniforms.f[95].wwww).x;
reg_tmp11.y = (-uniforms.f[95].wwww).y;
reg_tmp7.xz = (reg_tmp9.wwyy + -reg_tmp6.yyww).xz;
reg_tmp7.y = (reg_tmp6.xxxx + -reg_tmp9.zzzz).y;
reg_tmp7.w = (reg_tmp11).w;
reg_tmp11 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp11 = vec4(rsq_s(reg_tmp11.x));
vs_out_attr3 = mul_s(reg_tmp7, reg_tmp11);
if (uniforms.b[0]) {
sub_2();
} else {
sub_3();
}
vs_out_attr5 = reg_tmp7.xyzz;
if (uniforms.b[9]) {
sub_4();
} else {
sub_5();
}
reg_tmp10.x = dot_s(uniforms.f[8].wzyx, reg_tmp8);
reg_tmp10.y = dot_s(uniforms.f[9].wzyx, reg_tmp8);
reg_tmp10.z = dot_s(uniforms.f[10].wzyx, reg_tmp8);
vs_out_attr4 = -reg_tmp10;
vs_out_attr2 = vs_in_reg2.xyxy;
vs_out_attr1.xyz = (reg_tmp0).xyz;
vs_out_attr1.w = (mul_s(uniforms.f[94].zzzz, vs_in_reg1.wwww)).w;
return true;
}
bool sub_1() {
reg_tmp9.x = (-reg_tmp9.wwww).x;
return false;
}
bool sub_2() {
reg_tmp4 = -uniforms.f[12].wzyx + reg_tmp8;
reg_tmp4.w = (uniforms.f[95].yyyy).w;
reg_tmp5.x = dot_s(reg_tmp4, reg_tmp4);
reg_tmp5.x = rsq_s(reg_tmp5.x);
reg_tmp5 = mul_s(reg_tmp4, reg_tmp5.xxxx);
reg_tmp7.x = dot_s(uniforms.f[13].wzyx, reg_tmp5);
reg_tmp7.y = dot_s(uniforms.f[14].wzyx, reg_tmp5);
reg_tmp7.z = dot_s(uniforms.f[15].wzyx, reg_tmp5);
return false;
}
bool sub_3() {
reg_tmp7.x = dot_s(uniforms.f[13].wzyx, reg_tmp8);
reg_tmp7.y = dot_s(uniforms.f[14].wzyx, reg_tmp8);
reg_tmp7.z = dot_s(uniforms.f[15].wzyx, reg_tmp8);
return false;
}
bool sub_4() {
reg_tmp5 = mul_s(uniforms.f[95].xxxx, vs_in_reg3);
reg_tmp1 = uniforms.f[94].wwww + -reg_tmp5;
reg_tmp2.x = rcp_s(reg_tmp1.x);
reg_tmp2.y = rcp_s(reg_tmp1.y);
reg_tmp2.z = rcp_s(reg_tmp1.z);
reg_tmp5.xyz = (mul_s(reg_tmp5, reg_tmp2)).xyz;
reg_tmp0.xyz = (reg_tmp5.xyzz).xyz;
reg_tmp0.w = (reg_tmp5.wwww).w;
return false;
}
bool sub_5() {
reg_tmp0 = uniforms.f[95].wwww;
return false;
}
// reference: 9F7C05AE84FC9B44, 10CD3F452451A509
// shader: 8B30, 939992873A2D3B42
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 336D534DD7B890BD, 939992873A2D3B42
// program: 10CD3F452451A509, DADA7DA78218FFEA, 939992873A2D3B42
// shader: 8B30, F5F6FC31E29885A0
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 04FEB66CD7B890BD, F5F6FC31E29885A0
// program: 10CD3F452451A509, DADA7DA78218FFEA, F5F6FC31E29885A0
// shader: 8DD9, AF3D45D1FD56EDEB

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

    texcoord0 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);
    texcoord1 = vec2(0.0, 0.0);

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
// reference: 5144ABDD19096853, AF3D45D1FD56EDEB
// shader: 8B31, BB8DFD8D55A698E1

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
vs_out_attr0.xy = (vs_in_reg0.xyyy).xy;
vs_out_attr0.zw = (uniforms.f[90].wzwz).zw;
vs_out_attr1 = vs_in_reg0.zwzw;
reg_tmp0 = uniforms.f[5].wzyx;
vs_out_attr2 = reg_tmp0.xxxx;
return true;
}
// reference: 57AC5B666C98EFDF, BB8DFD8D55A698E1
// shader: 8B30, 9F17872FE570DCEE
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp(vec3(dot((last_tex_env_out.rgb) - vec3(0.5), (const_color[1].rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_1 = color_output_1[0];
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp(vec3(dot((combiner_buffer.rgb) - vec3(0.5), (const_color[2].rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) + (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp(vec3(dot((combiner_buffer.rgb) - vec3(0.5), (const_color[3].rgb) - vec3(0.5)) * 4.0), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.r) + (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.aaa) * (const_color[4].rgb) + (last_tex_env_out.aaa) * (vec3(1) - (const_color[4].rgb)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.r) + (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb) + (last_tex_env_out.aaa) * (vec3(1) - (const_color[5].rgb)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5 * 4.0, alpha_output_5 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A3662B932D135FFA, 9F17872FE570DCEE
// program: BB8DFD8D55A698E1, AF3D45D1FD56EDEB, 9F17872FE570DCEE
// shader: 8B30, 4EB1DA4CF9B0C0C9
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb) + (texcolor0.rgb) * (vec3(1) - (const_color[1].rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb) + (combiner_buffer.rgb) * (vec3(1) - (const_color[3].rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb) + (combiner_buffer.rgb) * (vec3(1) - (const_color[5].rgb)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7B076EE787F30E2F, 4EB1DA4CF9B0C0C9
// program: BB8DFD8D55A698E1, AF3D45D1FD56EDEB, 4EB1DA4CF9B0C0C9
// shader: 8B31, 43CFDD5AE4080F29

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
vs_out_attr0.xy = (vs_in_reg0.xyyy).xy;
vs_out_attr0.zw = (uniforms.f[90].wzwz).zw;
vs_out_attr1.xy = (vs_in_reg0.zwzw).xy;
vs_out_attr1.zw = (uniforms.f[5].wzwz + vs_in_reg0.zwzw).zw;
return true;
}
// reference: CDCDB47604222238, 43CFDD5AE4080F29
// shader: 8B30, B9882CC707E23D8B
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
vec3 color_output_0 = byteround(clamp((texcolor1.rrr) - (texcolor0.rrr), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor0.rrr) - (texcolor1.rrr), vec3(0), vec3(1)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (texcolor0.a);
last_tex_env_out = vec4(color_output_2 * 4.0, alpha_output_2 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3 * 4.0, alpha_output_3 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4 * 4.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.r) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5 * 1.0, alpha_output_5 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BE55333358038893, B9882CC707E23D8B
// program: 43CFDD5AE4080F29, B2B7BAAB97E6D825, B9882CC707E23D8B
// shader: 8B30, 43862000C693B7C1
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
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor1.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ABD90204FD5DCBEB, 43862000C693B7C1
// program: 43CFDD5AE4080F29, B2B7BAAB97E6D825, 43862000C693B7C1
// shader: 8DD9, DF2B9EAF579002C2

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
    texcoord2 = vec2(vtx.attributes[1].z, vtx.attributes[1].w);

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
// reference: F155A33764C29FD9, DF2B9EAF579002C2
// shader: 8B31, AD73FED42711C07F

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
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;

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
bool sub_1();
bool sub_2();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp1 = vs_in_reg0.xyzz;
reg_tmp1.w = (uniforms.f[95].wwww).w;
reg_tmp4 = mul_s(uniforms.f[92].wzyx, vs_in_reg3);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = mul_s(uniforms.f[5 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp13 = mul_s(uniforms.f[6 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp14 = mul_s(uniforms.f[7 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp12 = fma_s(reg_tmp4.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
if (uniforms.b[11]) {
sub_1();
}
if (uniforms.b[12]) {
sub_2();
}
reg_tmp8.x = dot_s(reg_tmp1, reg_tmp12);
reg_tmp8.y = dot_s(reg_tmp1, reg_tmp13);
reg_tmp8.z = dot_s(reg_tmp1, reg_tmp14);
reg_tmp8.w = (uniforms.f[95].wwww).w;
vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp8);
vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp8);
vs_out_attr0.z = dot_s(-uniforms.f[2].wzyx, reg_tmp8);
vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp8);
reg_tmp3 = uniforms.f[77].wzyx;
reg_tmp4 = fma_s(vs_in_reg2.xyxy, uniforms.f[78].wzyx, reg_tmp3);
vs_out_attr1 = reg_tmp4;
return true;
}
bool sub_1() {
reg_tmp4 = mul_s(uniforms.f[92].wzyx, vs_in_reg5);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = fma_s(reg_tmp4.zzzz, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.zzzz, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.zzzz, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp4.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
return false;
}
bool sub_2() {
reg_tmp4 = mul_s(uniforms.f[95].zzzz, vs_in_reg6);
reg_tmp5 = mul_s(uniforms.f[95].yyyy, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = fma_s(reg_tmp5.xxxx, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.xxxx, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.xxxx, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp5.yyyy, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.yyyy, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.yyyy, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
addr_regs.xy = ivec2(reg_tmp4.zw);
reg_tmp12 = fma_s(reg_tmp5.zzzz, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.zzzz, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.zzzz, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp5.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
return false;
}
// reference: 5DF804C78E7C8D46, AD73FED42711C07F
// shader: 8B30, D13EA8CA11590A72
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;

uint d = uint(clamp(depth, 0.0, 1.0) * (exp2(24.0) - 1.0));
uint s = uint(last_tex_env_out.g * 255.0);
ivec2 image_coord = ivec2(gl_FragCoord.xy);
uint old = imageLoad(shadow_buffer, image_coord).x;
uint new;
uint old2;
do {
    old2 = old;
    uvec2 ref = DecodeShadow(old);
    if (d < ref.x) {
        if (s == 0u) {
            ref.x = d;
        } else {
            s = uint(float(s) / (shadow_bias_constant + shadow_bias_linear * float(d) / float(ref.x)));
            ref.y = min(s, ref.y);
        }
    }
    new = EncodeShadow(ref);
} while ((old = imageAtomicCompSwap(shadow_buffer, image_coord, old, new)) != old2);
}
// reference: 37ED09C41E8EF84D, D13EA8CA11590A72
// program: AD73FED42711C07F, DF2B9EAF579002C2, D13EA8CA11590A72
// shader: 8B31, B710898567ABFB66

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
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;

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
reg_tmp2.xy = (uniforms.f[95].wwww).xy;
reg_tmp2.zw = (uniforms.f[95].zzzz).zw;
reg_tmp1 = vs_in_reg0.xyzz;
reg_tmp1.w = (uniforms.f[95].yyyy).w;
reg_tmp4 = mul_s(vs_in_reg3, reg_tmp2);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = mul_s(uniforms.f[5 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp13 = mul_s(uniforms.f[6 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp14 = mul_s(uniforms.f[7 + addr_regs.x].wzyx, reg_tmp4.zzzz);
reg_tmp12 = fma_s(reg_tmp4.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
if (uniforms.b[11]) {
sub_1();
}
reg_tmp8.x = dot_s(reg_tmp1, reg_tmp12);
reg_tmp8.y = dot_s(reg_tmp1, reg_tmp13);
reg_tmp8.z = dot_s(reg_tmp1, reg_tmp14);
reg_tmp8.w = (uniforms.f[95].yyyy).w;
reg_tmp9.x = dot_s(uniforms.f[0].wzyx, reg_tmp8);
reg_tmp9.y = dot_s(uniforms.f[1].wzyx, reg_tmp8);
reg_tmp9.z = dot_s(-uniforms.f[2].wzyx, reg_tmp8);
reg_tmp9.w = dot_s(uniforms.f[3].wzyx, reg_tmp8);
reg_tmp10.x = (uniforms.f[95].yyyy).x;
reg_tmp10.x = (-uniforms.f[95].xxxx + reg_tmp10.xxxx).x;
reg_tmp10.y = (uniforms.f[95].yyyy).y;
reg_tmp2.xy = (mul_s(-reg_tmp9.wwww, reg_tmp10.xyyy)).xy;
bool_regs.x = reg_tmp9.xxxx.x < reg_tmp2.xyyy.x;
bool_regs.y = reg_tmp9.xxxx.y > reg_tmp2.xyyy.y;
if (all(bool_regs)) {
sub_3();
}
vs_out_attr0 = reg_tmp9;
reg_tmp10 = vs_in_reg1.xyzz;
reg_tmp9.x = dot_3(reg_tmp10.xyz, reg_tmp12.xyz);
reg_tmp9.y = dot_3(reg_tmp10.xyz, reg_tmp13.xyz);
reg_tmp9.z = dot_3(reg_tmp10.xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_3(uniforms.f[77].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[78].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[79].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[94].wwww).w;
reg_tmp6.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp6 = mul_s(reg_tmp2, reg_tmp6.xxxx);
reg_tmp6.w = (uniforms.f[95].yyyy).w;
if (uniforms.b[8]) {
sub_4();
} else {
sub_5();
}
if (uniforms.b[0]) {
sub_8();
} else {
sub_9();
}
vs_out_attr5 = reg_tmp7.xyzz;
reg_tmp10.x = dot_s(uniforms.f[77].wzyx, reg_tmp8);
reg_tmp10.y = dot_s(uniforms.f[78].wzyx, reg_tmp8);
reg_tmp10.z = dot_s(uniforms.f[79].wzyx, reg_tmp8);
vs_out_attr3 = -reg_tmp10;
vs_out_attr1 = vs_in_reg2.xyxy;
vs_out_attr4.xyz = (uniforms.f[94].wwww).xyz;
vs_out_attr4.w = (mul_s(uniforms.f[94].yyyy, vs_in_reg1.wwww)).w;
return true;
}
bool sub_1() {
reg_tmp4 = mul_s(vs_in_reg5, reg_tmp2);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = fma_s(reg_tmp4.zzzz, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.zzzz, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.zzzz, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp4.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp4.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp4.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
if (uniforms.b[12]) {
sub_2();
}
return false;
}
bool sub_2() {
reg_tmp4 = mul_s(uniforms.f[95].wwww, vs_in_reg6);
reg_tmp5 = mul_s(uniforms.f[95].zzzz, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp4.xy);
reg_tmp12 = fma_s(reg_tmp5.xxxx, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.xxxx, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.xxxx, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp5.yyyy, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.yyyy, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.yyyy, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
addr_regs.xy = ivec2(reg_tmp4.zw);
reg_tmp12 = fma_s(reg_tmp5.zzzz, uniforms.f[5 + addr_regs.x].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.zzzz, uniforms.f[6 + addr_regs.x].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.zzzz, uniforms.f[7 + addr_regs.x].wzyx, reg_tmp14);
reg_tmp12 = fma_s(reg_tmp5.wwww, uniforms.f[5 + addr_regs.y].wzyx, reg_tmp12);
reg_tmp13 = fma_s(reg_tmp5.wwww, uniforms.f[6 + addr_regs.y].wzyx, reg_tmp13);
reg_tmp14 = fma_s(reg_tmp5.wwww, uniforms.f[7 + addr_regs.y].wzyx, reg_tmp14);
return false;
}
bool sub_3() {
reg_tmp9.x = (-reg_tmp9.wwww).x;
return false;
}
bool sub_4() {
reg_tmp10 = vs_in_reg4.xyzz;
reg_tmp9.x = dot_3(reg_tmp10.xyz, reg_tmp12.xyz);
reg_tmp9.y = dot_3(reg_tmp10.xyz, reg_tmp13.xyz);
reg_tmp9.z = dot_3(reg_tmp10.xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_3(uniforms.f[77].wzy, reg_tmp9.xyz);
reg_tmp2.y = dot_3(uniforms.f[78].wzy, reg_tmp9.xyz);
reg_tmp2.z = dot_3(uniforms.f[79].wzy, reg_tmp9.xyz);
reg_tmp2.w = (uniforms.f[94].wwww).w;
reg_tmp9.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp9.x = rsq_s(reg_tmp9.x);
reg_tmp9 = mul_s(reg_tmp2, reg_tmp9.xxxx);
reg_tmp9.w = (uniforms.f[95].yyyy).w;
reg_tmp7 = mul_s(reg_tmp6.yzxx, reg_tmp9.zxyy);
reg_tmp7 = fma_s(-reg_tmp9.yzxx, reg_tmp6.zxyy, reg_tmp7);
reg_tmp7.w = dot_3(reg_tmp7.xyz, reg_tmp7.xyz);
reg_tmp7.w = rsq_s(reg_tmp7.w);
reg_tmp7 = mul_s(reg_tmp7, reg_tmp7.wwww);
reg_tmp11.w = (reg_tmp6.zzzz + reg_tmp7.yyyy).w;
reg_tmp9 = mul_s(reg_tmp7.yzxx, reg_tmp6.zxyy);
reg_tmp9 = fma_s(-reg_tmp6.yzxx, reg_tmp7.zxyy, reg_tmp9);
reg_tmp11.w = (reg_tmp9.xxxx + reg_tmp11).w;
reg_tmp9.w = (reg_tmp7.zzzz).w;
reg_tmp7.z = (reg_tmp9.xxxx).z;
reg_tmp11.w = (uniforms.f[95].yyyy + reg_tmp11).w;
reg_tmp6.w = (reg_tmp7.xxxx).w;
reg_tmp7.x = (reg_tmp6.zzzz).x;
reg_tmp11.x = (uniforms.f[95].yyyy).x;
reg_tmp11.y = (-uniforms.f[95].yyyy).y;
reg_tmp7.xz = (reg_tmp9.wwyy + -reg_tmp6.yyww).xz;
reg_tmp7.y = (reg_tmp6.xxxx + -reg_tmp9.zzzz).y;
reg_tmp7.w = (reg_tmp11).w;
reg_tmp11 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp11 = vec4(rsq_s(reg_tmp11.x));
vs_out_attr2 = mul_s(reg_tmp7, reg_tmp11);
return false;
}
bool sub_5() {
bool_regs = equal(-uniforms.f[95].yy, reg_tmp6.zz);
reg_tmp4 = uniforms.f[95].yyyy + reg_tmp6.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
vs_out_attr2.w = (uniforms.f[94].wwww).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp6);
if (!bool_regs.x) {
sub_6();
} else {
sub_7();
}
return false;
}
bool sub_6() {
vs_out_attr2.z = rcp_s(reg_tmp4.x);
vs_out_attr2.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_7() {
vs_out_attr2.x = (uniforms.f[95].yyyy).x;
vs_out_attr2.yzw = (uniforms.f[94].wwww).yzw;
return false;
}
bool sub_8() {
reg_tmp4 = -uniforms.f[81].wzyx + reg_tmp8;
reg_tmp4.w = (uniforms.f[94].wwww).w;
reg_tmp5.x = dot_s(reg_tmp4, reg_tmp4);
reg_tmp5.x = rsq_s(reg_tmp5.x);
reg_tmp5 = mul_s(reg_tmp4, reg_tmp5.xxxx);
reg_tmp7.x = dot_s(uniforms.f[82].wzyx, reg_tmp5);
reg_tmp7.y = dot_s(uniforms.f[83].wzyx, reg_tmp5);
reg_tmp7.z = dot_s(uniforms.f[84].wzyx, reg_tmp5);
return false;
}
bool sub_9() {
reg_tmp7.x = dot_s(uniforms.f[82].wzyx, reg_tmp8);
reg_tmp7.y = dot_s(uniforms.f[83].wzyx, reg_tmp8);
reg_tmp7.z = dot_s(uniforms.f[84].wzyx, reg_tmp8);
return false;
}
// reference: 306A2E36216E2D81, B710898567ABFB66
// shader: 8B30, 41967151203712D2
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - rounded_primary_color.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7FDC11CEAFA6D528, 41967151203712D2
// program: B710898567ABFB66, 1E9B8787AEA67ABF, 41967151203712D2
// shader: 8B30, A480AE4BE233C30A
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4F00266CC2586438, A480AE4BE233C30A
// program: 10CD3F452451A509, DADA7DA78218FFEA, A480AE4BE233C30A
// shader: 8B30, 2B434053B4DE3312
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec3 color_output_0 = byteround(clamp((vec3(1) - shadowTexture(texcoord0, texcoord0_w).rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((shadowTexture(texcoord0, texcoord0_w).rgb) * (texcolor1.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 49A76F833596CCB8, 2B434053B4DE3312
// program: 10CD3F452451A509, DADA7DA78218FFEA, 2B434053B4DE3312
// shader: 8B30, C75B2661DFD74194
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - rounded_primary_color.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 484FF4EFAFA6D528, C75B2661DFD74194
// program: B710898567ABFB66, 1E9B8787AEA67ABF, C75B2661DFD74194
// shader: 8B30, 6837A7620189D676
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7893C34DC2586438, 6837A7620189D676
// program: 10CD3F452451A509, DADA7DA78218FFEA, 6837A7620189D676
// reference: 21DFFB4900EF9A82, DDA61FA7AF3B4489
// reference: A2A441A42D135FFA, 9F17872FE570DCEE
// reference: 7AC504D087F30E2F, 4EB1DA4CF9B0C0C9
// reference: BF97590458038893, B9882CC707E23D8B
// reference: AA1B6833FD5DCBEB, 43862000C693B7C1
// reference: 7893C34DF2E07B73, 6837A7620189D676
// reference: C4AFFEA10BC51C0D, 45E2174AE15FBF0E
// reference: C4AFFEA1B367F46D, 887808CD0BBA6904
// reference: C7933D4FC3B71EAB, 4C9684C02DD9FF08
// reference: F33C1B800ADAFCB5, 96B45B900F118596
// reference: 2E08FC6781BA9229, 1EE4DB54AED21359
// reference: 362F63F31E8EF84D, D13EA8CA11590A72
// shader: 8B30, 98BC708860E36D2E
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;

uint d = uint(clamp(depth, 0.0, 1.0) * (exp2(24.0) - 1.0));
uint s = uint(last_tex_env_out.g * 255.0);
ivec2 image_coord = ivec2(gl_FragCoord.xy);
uint old = imageLoad(shadow_buffer, image_coord).x;
uint new;
uint old2;
do {
    old2 = old;
    uvec2 ref = DecodeShadow(old);
    if (d < ref.x) {
        if (s == 0u) {
            ref.x = d;
        } else {
            s = uint(float(s) / (shadow_bias_constant + shadow_bias_linear * float(d) / float(ref.x)));
            ref.y = min(s, ref.y);
        }
    }
    new = EncodeShadow(ref);
} while ((old = imageAtomicCompSwap(shadow_buffer, image_coord, old, new)) != old2);
}
// reference: 7C1399C43BD61383, 98BC708860E36D2E
// program: AD73FED42711C07F, DF2B9EAF579002C2, 98BC708860E36D2E
// shader: 8B30, 9CA06CC7FB22C6E4
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = (secondary_fragment_color.b);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (texcolor1.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (const_color[5].aaa) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 401B2E218D03F207, 9CA06CC7FB22C6E4
// program: B710898567ABFB66, 1E9B8787AEA67ABF, 9CA06CC7FB22C6E4
// reference: C4AFFEA17A0A1673, A16D42D83E64F5B0
// shader: 8DD9, E957B9D48F28DB0E

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
struct Vertex {
    vec4 attributes[1];
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
    texcoord1 = vec2(0.0, 0.0);

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
    prim_buffer[0].attributes = vec4[1](vs_out_attr0[0]);
    prim_buffer[1].attributes = vec4[1](vs_out_attr0[1]);
    prim_buffer[2].attributes = vec4[1](vs_out_attr0[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: B0EECBB083D64E3E, E957B9D48F28DB0E
// shader: 8B31, D5CF1E60CB47008C

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

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
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
reg_tmp0 = vs_in_reg0.xyyy;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
reg_tmp0.z = (uniforms.f[95].wwww).z;
vs_out_attr0 = reg_tmp0;
return true;
}
// reference: 790A0EF8E8B04975, D5CF1E60CB47008C
// shader: 8B30, BE0EFAD295AC1477
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C4AFFEA1DDF5402E, BE0EFAD295AC1477
// program: D5CF1E60CB47008C, E957B9D48F28DB0E, BE0EFAD295AC1477
// shader: 8B31, 38B8B7FD780AB715

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
reg_tmp0 = vs_in_reg0.xyyy;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
vs_out_attr0 = reg_tmp0;
vs_out_attr1 = vs_in_reg0.zwzw;
return true;
}
// reference: 050560232FD9F576, 38B8B7FD780AB715
// shader: 8B30, 0450CB28C732E358
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
color = vec4(0);
}// reference: C4AFFEA115E45418, 0450CB28C732E358
// program: 38B8B7FD780AB715, B2B7BAAB97E6D825, 0450CB28C732E358
// shader: 8B31, C4B97BEFF97B7F5C

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0 = vs_in_reg0.xyyy;
reg_tmp0.zw = (uniforms.f[90].wzwz).zw;
vs_out_attr0 = reg_tmp0;
reg_tmp1 = vs_in_reg0.zwzw;
reg_tmp2 = vs_in_reg0.zwzw;
reg_tmp1 = uniforms.f[5].wzyx + reg_tmp1;
reg_tmp2 = uniforms.f[6].wzyx + reg_tmp2;
vs_out_attr1 = reg_tmp1;
vs_out_attr2 = reg_tmp2;
return true;
}
// reference: 7A1A4D2DA0E87D19, C4B97BEFF97B7F5C
// shader: 8B30, 83F8851BF3541A9C
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb) + (texcolor1.rgb) * (vec3(1) - (const_color[0].rgb)), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 77B03994A37D933D, 83F8851BF3541A9C
// program: C4B97BEFF97B7F5C, 48795C23CA84C4B5, 83F8851BF3541A9C
// shader: 8B30, 4BDCF544621654C2
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
vec4 texcolor2 = textureLod(tex2, texcoord1, getLod(texcoord1 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor2.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9EB44EFCF8B4258C, 4BDCF544621654C2
// program: 38B8B7FD780AB715, B2B7BAAB97E6D825, 4BDCF544621654C2
// reference: C4AFFEA19C2945DE, A16D42D83E64F5B0
// reference: C4AFFEA16A7FE2DF, 2DE87652989F51F5
// shader: 8B31, AB51D517F9435B27

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

bool sub_0();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
uint jmp_to = 0u;
while (true) {
switch (jmp_to) {
case 0u:
reg_tmp14.z = (mul_s(uniforms.f[5].wwww, vs_in_reg2.zzzz)).z;
reg_tmp15 = uniforms.f[6].wzyx;
reg_tmp0 = vs_in_reg0.xyzz;
reg_tmp0.z = (reg_tmp0.zzzz + reg_tmp14.zzzz).z;
reg_tmp0.w = (uniforms.f[95].wwww).w;
vs_out_attr1 = mul_s(uniforms.f[90].zzzz, vs_in_reg1);
bool_regs = greaterThan(uniforms.f[7].wz, reg_tmp15.xy);
if (bool_regs.y) {
jmp_to = 20u; break;
}
vs_out_attr0.x = dot_s(uniforms.f[8].wzyx, reg_tmp0);
vs_out_attr0.y = dot_s(uniforms.f[9].wzyx, reg_tmp0);
vs_out_attr0.z = dot_s(uniforms.f[10].wzyx, reg_tmp0);
vs_out_attr0.w = dot_s(uniforms.f[11].wzyx, reg_tmp0);
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
return true;
case 20u:
vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp0);
vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[3].wzyx, reg_tmp0);
reg_tmp4.x = (mul_s(uniforms.f[12].yyyy, reg_tmp14.zzzz)).x;
reg_tmp4.y = (mul_s(uniforms.f[13].yyyy, reg_tmp14.zzzz)).y;
reg_tmp4.z = (mul_s(uniforms.f[14].yyyy, reg_tmp14.zzzz)).z;
reg_tmp4.xyz = (reg_tmp4.xyzz + reg_tmp0.xyzz).xyz;
reg_tmp4.w = (uniforms.f[90].xxxx).w;
reg_tmp6.w = dot_s(uniforms.f[3].wzyx, reg_tmp4);
reg_tmp6.z = dot_s(-uniforms.f[2].wzyx, reg_tmp4);
reg_tmp6.z = (mul_s(reg_tmp1.wwww, reg_tmp6.zzzz)).z;
reg_tmp2 = vec4(rcp_s(reg_tmp6.w));
vs_out_attr0.z = (mul_s(reg_tmp6.zzzz, reg_tmp2.zzzz)).z;
vs_out_attr0.w = (reg_tmp1.wwww).w;
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
return true;
default: return false;
}
}
return false;
}
bool sub_1() {
reg_tmp1.xy = (vs_in_reg2.xyyy).xy;
reg_tmp1.xy = (mul_s(uniforms.f[90].yyyy, reg_tmp1.xyyy)).xy;
reg_tmp1.y = (uniforms.f[95].wwww + -reg_tmp1.yyyy).y;
vs_out_attr2 = reg_tmp1.xyyy;
return false;
}
bool sub_2() {
vs_out_attr2 = uniforms.f[90].wwww;
return false;
}
bool sub_3() {
reg_tmp5.xy = (vs_in_reg2.xyyy).xy;
reg_tmp5.xy = (mul_s(uniforms.f[90].yyyy, reg_tmp5.xyyy)).xy;
reg_tmp5.y = (uniforms.f[95].wwww + -reg_tmp5.yyyy).y;
vs_out_attr2 = reg_tmp5.xyyy;
return false;
}
bool sub_4() {
vs_out_attr2 = uniforms.f[90].wwww;
return false;
}
// reference: A9EC512CC577D63E, AB51D517F9435B27
// program: AB51D517F9435B27, 8F66AB8FEF098E16, 4C9684C02DD9FF08
// reference: B73F87094F4A5AEC, 495B70B9ABF582BB
// shader: 8B30, FEAD9BDB63FFC15E
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AD6D2F944F4A5AEC, FEAD9BDB63FFC15E
// program: F2EFC8AD043F300B, DADA7DA78218FFEA, FEAD9BDB63FFC15E
// reference: 2A08FBE699E508E6, D13EA8CA11590A72
// shader: 8B30, C341531AD0964728
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - rounded_primary_color.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 63FB89DB28CD2583, C341531AD0964728
// program: B710898567ABFB66, 1E9B8787AEA67ABF, C341531AD0964728
// reference: 5327BE7945339493, A480AE4BE233C30A
// reference: 5580F796B2FD3C13, 2B434053B4DE3312
// reference: 64B45B5845339493, 6837A7620189D676
// shader: 8B30, A144000A1DFBD457
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - rounded_primary_color.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 54686CFA28CD2583, A144000A1DFBD457
// program: B710898567ABFB66, 1E9B8787AEA67ABF, A144000A1DFBD457
// reference: 64B45B58758B8BD8, 6837A7620189D676
// shader: 8B30, 44168E49FBAAAD21
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
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C7933D4F99ACAEF6, 44168E49FBAAAD21
// program: 22B227623BC2B584, BD6EFF0A593527B3, 44168E49FBAAAD21
// shader: 8DD9, DA13D1C10694E19D

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)
layout(points) in;
layout(triangle_strip, max_vertices=30) out;
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
layout(location=7) in vec4 vs_out_attr7[];

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms gs_uniforms
layout(binding=3, std140) uniform gs_config {
    pica_uniforms uniforms;
};
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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);
    texcoord1 = vec2(0.0, 0.0);

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

Vertex output_buffer;
Vertex prim_buffer[3];
uint vertex_id = 0u;
bool prim_emit = false;
bool winding = false;
void setemit(uint vertex_id_, bool prim_emit_, bool winding_);
void emit();
void main() {
    output_buffer.attributes[0] = vec4(0.0, 0.0, 0.0, 1.0);
    output_buffer.attributes[1] = vec4(0.0, 0.0, 0.0, 1.0);
    output_buffer.attributes[2] = vec4(0.0, 0.0, 0.0, 1.0);
    exec_shader();
}

void setemit(uint vertex_id_, bool prim_emit_, bool winding_) {
    vertex_id = vertex_id_;
    prim_emit = prim_emit_;
    winding = winding_;
}
void emit() {
    prim_buffer[vertex_id] = output_buffer;
    if (prim_emit) {
        if (winding) {
            EmitPrim(prim_buffer[1], prim_buffer[0], prim_buffer[2]);
            winding = false;
        } else {
            EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
        }
    }
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
bool sub_14();
bool sub_15();
bool sub_12();
bool sub_13();
bool sub_9();
bool sub_10();
bool sub_6();
bool sub_7();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_11();
bool sub_8();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
uint jmp_to = 60u;
while (true) {
switch (jmp_to) {
case 60u:
bool_regs = lessThan(uniforms.f[82].zz, vs_out_attr3[0].ww);
if (bool_regs.x) {
jmp_to = 130u; break;
}
reg_tmp1.xy = (reg_tmp15.xxxx).xy;
bool_regs = lessThan(uniforms.f[81].xx, reg_tmp1.xy);
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
bool_regs.x = uniforms.f[81].xxxx.x == reg_tmp15.wwww.x;
bool_regs.y = uniforms.f[81].xxxx.y != reg_tmp15.wwww.y;
if (bool_regs.x) {
jmp_to = 97u; break;
}
bool_regs = equal(uniforms.f[81].zw, reg_tmp15.ww);
if (bool_regs.x) {
jmp_to = 108u; break;
}
if (bool_regs.y) {
jmp_to = 119u; break;
}
reg_tmp6.xyz = (reg_tmp0.xyzz).xyz;
sub_3();
output_buffer.attributes[2].xy = (vs_out_attr2[0].xyyy).xy;
sub_6();
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
sub_3();
output_buffer.attributes[2].xy = (vs_out_attr2[0].zwww).xy;
sub_7();
sub_8();
return true;
case 97u:
reg_tmp6.xyz = (reg_tmp0.xyzz).xyz;
sub_3();
output_buffer.attributes[2].xy = (vs_out_attr2[0].xyyy).xy;
sub_14();
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
sub_3();
output_buffer.attributes[2].xy = (vs_out_attr2[0].zwww).xy;
sub_15();
sub_11();
return true;
case 108u:
reg_tmp6.xyz = (reg_tmp0.xyzz).xyz;
sub_3();
output_buffer.attributes[2].xy = (vs_out_attr2[0].xyyy).xy;
sub_12();
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
sub_3();
output_buffer.attributes[2].xy = (vs_out_attr2[0].zwww).xy;
sub_13();
sub_11();
return true;
case 119u:
reg_tmp6.xyz = (reg_tmp0.xyzz).xyz;
sub_3();
output_buffer.attributes[2].xy = (vs_out_attr2[0].xyyy).xy;
sub_9();
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
sub_3();
output_buffer.attributes[2].xy = (vs_out_attr2[0].zwww).xy;
sub_10();
sub_11();
return true;
case 130u:
reg_tmp11 = vs_out_attr4[0];
reg_tmp12 = vs_out_attr5[0];
reg_tmp13 = vs_out_attr6[0];
reg_tmp14 = vs_out_attr7[0];
reg_tmp15.w = (uniforms.f[81].xxxx).w;
reg_tmp15.z = (vs_out_attr0[0].wwww).z;
reg_tmp15.x = (vs_out_attr3[0].zzzz).x;
return true;
default: return false;
}
}
return false;
}
bool sub_1() {
reg_tmp0 = vs_out_attr3[0].xyzz;
reg_tmp0.w = (uniforms.f[81].xxxx).w;
reg_tmp0 = mul_s(reg_tmp0, vs_out_attr4[0].wwww);
return false;
}
bool sub_2() {
reg_tmp0.xyz = (vs_out_attr4[0].xyzz).xyz;
reg_tmp1.xyz = (vs_out_attr3[0].xyzz).xyz;
reg_tmp2.xyz = (mul_s(reg_tmp0.yzxx, reg_tmp1.zxyy)).xyz;
reg_tmp2.xyz = (fma_s(-reg_tmp1.yzxx, reg_tmp0.zxyy, reg_tmp2)).xyz;
reg_tmp2.w = (uniforms.f[81].xxxx).w;
reg_tmp0.x = dot_s(reg_tmp2, reg_tmp2);
reg_tmp0.x = rsq_s(reg_tmp0.x);
reg_tmp0 = mul_s(reg_tmp2, reg_tmp0.xxxx);
reg_tmp0.xyz = (mul_s(reg_tmp0.xyzz, vs_out_attr4[0].wwww)).xyz;
reg_tmp7.x = (vs_out_attr5[0].zzzz).x;
reg_tmp7.y = (vs_out_attr6[0].zzzz).y;
reg_tmp7.z = (vs_out_attr7[0].zzzz).z;
reg_tmp7.xyz = (mul_s(reg_tmp7.xyzz, reg_tmp15.zzzz)).xyz;
return false;
}
bool sub_14() {
setemit(0u, false, false);
emit();
return false;
}
bool sub_15() {
setemit(1u, false, false);
emit();
return false;
}
bool sub_12() {
setemit(2u, true, false);
emit();
return false;
}
bool sub_13() {
setemit(0u, true, true);
emit();
return false;
}
bool sub_9() {
setemit(1u, true, false);
emit();
return false;
}
bool sub_10() {
setemit(2u, true, true);
emit();
return false;
}
bool sub_6() {
setemit(0u, true, false);
emit();
return false;
}
bool sub_7() {
setemit(1u, true, true);
emit();
return false;
}
bool sub_3() {
reg_tmp1.xy = (reg_tmp15.xxxx).xy;
bool_regs = lessThan(uniforms.f[81].xx, reg_tmp1.xy);
if (bool_regs.x) {
sub_4();
} else {
sub_5();
}
output_buffer.attributes[1] = vs_out_attr1[0];
return false;
}
bool sub_4() {
reg_tmp5.xyz = (vs_out_attr0[0].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp5.w = (uniforms.f[81].zzzz).w;
output_buffer.attributes[0].x = dot_s(reg_tmp5, reg_tmp11);
output_buffer.attributes[0].y = dot_s(reg_tmp5, reg_tmp12);
output_buffer.attributes[0].z = dot_s(reg_tmp5, reg_tmp13);
output_buffer.attributes[0].w = dot_s(reg_tmp5, reg_tmp14);
return false;
}
bool sub_5() {
reg_tmp5.xyz = (vs_out_attr0[0].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp5.w = (uniforms.f[81].zzzz).w;
output_buffer.attributes[0].x = dot_s(reg_tmp5, reg_tmp11);
output_buffer.attributes[0].y = dot_s(reg_tmp5, reg_tmp12);
reg_tmp1.w = dot_s(reg_tmp5, reg_tmp14);
reg_tmp2.xyz = (reg_tmp7.xyzz + reg_tmp5.xyzz).xyz;
reg_tmp2.w = (uniforms.f[81].zzzz).w;
reg_tmp3.w = dot_s(reg_tmp2, reg_tmp14);
reg_tmp3.z = dot_s(reg_tmp2, -reg_tmp13);
reg_tmp3.z = (mul_s(reg_tmp1.wwww, reg_tmp3.zzzz)).z;
reg_tmp2 = vec4(rcp_s(reg_tmp3.w));
output_buffer.attributes[0].z = (mul_s(reg_tmp3.zzzz, reg_tmp2.zzzz)).z;
output_buffer.attributes[0].w = (reg_tmp1.wwww).w;
return false;
}
bool sub_11() {
reg_tmp15.w = (uniforms.f[81].zzzz + reg_tmp15.wwww).w;
return false;
}
bool sub_8() {
reg_tmp15.w = (uniforms.f[81].zzzz).w;
return false;
}
// reference: A61E5863A6182D0A, DA13D1C10694E19D
// shader: 8B31, E6E3C519ABB55164

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
layout(location=6) out vec4 vs_out_attr6;
layout(location=7) out vec4 vs_out_attr7;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
vs_out_attr7 = vec4(0, 0, 0, 1);
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

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
uint jmp_to = 0u;
while (true) {
switch (jmp_to) {
case 0u:
vs_out_attr0.xyz = (vs_in_reg0.xyzz).xyz;
vs_out_attr1 = mul_s(uniforms.f[90].yyyy, vs_in_reg1);
reg_tmp1.xy = (vs_in_reg4.xxxx).xy;
bool_regs = lessThan(uniforms.f[89].ww, reg_tmp1.xy);
if (bool_regs.x) {
jmp_to = 41u; break;
}
vs_out_attr0.w = (uniforms.f[90].wwww).w;
reg_tmp0 = uniforms.f[5].wzyx;
bool_regs = greaterThan(uniforms.f[6].wz, reg_tmp0.xy);
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
reg_tmp0 = uniforms.f[5].wzyx;
bool_regs = greaterThan(uniforms.f[6].wz, reg_tmp0.xy);
if (bool_regs.y) {
sub_3();
} else {
sub_4();
}
vs_out_attr3.w = (vs_in_reg4.xxxx).w;
reg_tmp2.x = (uniforms.f[89].xxxx + vs_in_reg2.zzzz).x;
reg_tmp3 = vec4(rcp_s(uniforms.f[91].w));
reg_tmp4.x = (uniforms.f[89].xxxx + vs_in_reg2.wwww).x;
reg_tmp4.x = (mul_s(reg_tmp4.xxxx, reg_tmp3.xxxx)).x;
reg_tmp2.x = (reg_tmp2.xxxx + reg_tmp4.xxxx).x;
reg_tmp2.x = (mul_s(uniforms.f[89].zzzz, reg_tmp2.xxxx)).x;
reg_tmp3.xyz = (-uniforms.f[8].wzyx + vs_in_reg0.xyzz).xyz;
vs_out_attr4.xyz = (reg_tmp3.xyzz).xyz;
vs_out_attr4.w = (reg_tmp2.xxxx).w;
vs_out_attr5 = uniforms.f[9].wzyx;
vs_out_attr6 = uniforms.f[10].wzyx;
vs_out_attr7 = uniforms.f[11].wzyx;
return true;
case 41u:
vs_out_attr0.w = (mul_s(uniforms.f[13].wwww, vs_in_reg2.zzzz)).w;
reg_tmp0 = uniforms.f[5].wzyx;
bool_regs = greaterThan(uniforms.f[6].wz, reg_tmp0.xy);
vs_out_attr2 = uniforms.f[90].wwww;
vs_out_attr3.xy = (uniforms.f[90].wwww).xy;
vs_out_attr3.w = (uniforms.f[89].yyyy).w;
if (bool_regs.y) {
sub_5();
} else {
sub_6();
}
return true;
default: return false;
}
}
return false;
}
bool sub_1() {
reg_tmp3.xy = (vs_in_reg2.xyyy).xy;
reg_tmp3.z = (vs_in_reg3.xxxx).z;
reg_tmp3.w = (vs_in_reg3.yyyy).w;
reg_tmp3 = mul_s(uniforms.f[7].wzwz, reg_tmp3);
reg_tmp3.y = (uniforms.f[90].xxxx + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[90].xxxx + -reg_tmp3.wwww).w;
vs_out_attr2 = reg_tmp3;
return false;
}
bool sub_2() {
vs_out_attr2 = uniforms.f[90].wwww;
return false;
}
bool sub_3() {
reg_tmp3.xyz = (vs_in_reg4.yzww).xyz;
vs_out_attr3.xyz = (mul_s(uniforms.f[90].zzzz, reg_tmp3.xyzz)).xyz;
return false;
}
bool sub_4() {
reg_tmp3.xy = (mul_s(uniforms.f[90].zzzz, vs_in_reg4.yzww)).xy;
vs_out_attr3.x = (mul_s(-uniforms.f[95].wwww, reg_tmp3.yyyy)).x;
vs_out_attr3.y = (reg_tmp3.xxxx).y;
vs_out_attr3.z = (uniforms.f[90].wwww).z;
return false;
}
bool sub_5() {
vs_out_attr3.z = (uniforms.f[90].wwww).z;
vs_out_attr4 = uniforms.f[0].wzyx;
vs_out_attr5 = uniforms.f[1].wzyx;
vs_out_attr6 = uniforms.f[2].wzyx;
vs_out_attr7 = uniforms.f[3].wzyx;
return false;
}
bool sub_6() {
vs_out_attr3.z = (uniforms.f[90].xxxx).z;
vs_out_attr4 = uniforms.f[14].wzyx;
vs_out_attr5 = uniforms.f[15].wzyx;
vs_out_attr6 = uniforms.f[16].wzyx;
vs_out_attr7 = uniforms.f[17].wzyx;
return false;
}
// reference: 71CE5DFE44B5D170, E6E3C519ABB55164
// program: E6E3C519ABB55164, DA13D1C10694E19D, 4C9684C02DD9FF08
// reference: 29F013F62E9D28AC, D13EA8CA11590A72
// shader: 8B30, 37E63426C59E9D02
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (refl_value * light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[4].position + view);
spot_dir = light_src[4].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[4].diffuse * dot_product) + light_src[4].ambient) * LookupLightingLUTUnsigned(20, clamp(light_src[4].dist_atten_scale * length(-light_src[4].position - view) + light_src[4].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[4].specular_0) + (refl_value * light_src[4].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(20, clamp(light_src[4].dist_atten_scale * length(-light_src[4].position - view) + light_src[4].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - rounded_primary_color.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 600361CB9FB505C9, 37E63426C59E9D02
// program: B710898567ABFB66, 1E9B8787AEA67ABF, 37E63426C59E9D02
// reference: 50DF5669F24BB4D9, A480AE4BE233C30A
// reference: 56781F8605851C59, 2B434053B4DE3312
// shader: 8B30, B66495AD9A4F4390
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (refl_value * light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[4].position + view);
spot_dir = light_src[4].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[4].diffuse * dot_product) + light_src[4].ambient) * LookupLightingLUTUnsigned(20, clamp(light_src[4].dist_atten_scale * length(-light_src[4].position - view) + light_src[4].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[4].specular_0) + (refl_value * light_src[4].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(20, clamp(light_src[4].dist_atten_scale * length(-light_src[4].position - view) + light_src[4].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (const_color[5].aaa) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5159445AB3CECDF7, B66495AD9A4F4390
// program: B710898567ABFB66, 1E9B8787AEA67ABF, B66495AD9A4F4390
// reference: 674CB348F24BB4D9, 6837A7620189D676
// shader: 8B30, 635F41CFDE9548C6
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (refl_value * light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[4].position + view);
spot_dir = light_src[4].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[4].diffuse * dot_product) + light_src[4].ambient) * LookupLightingLUTUnsigned(20, clamp(light_src[4].dist_atten_scale * length(-light_src[4].position - view) + light_src[4].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[4].specular_0) + (refl_value * light_src[4].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(20, clamp(light_src[4].dist_atten_scale * length(-light_src[4].position - view) + light_src[4].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - rounded_primary_color.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 579084EA9FB505C9, 635F41CFDE9548C6
// program: B710898567ABFB66, 1E9B8787AEA67ABF, 635F41CFDE9548C6
// reference: 674CB348C2F3AB92, 6837A7620189D676
// program: E6E3C519ABB55164, DA13D1C10694E19D, 44168E49FBAAAD21
// program: AB51D517F9435B27, 8F66AB8FEF098E16, 44168E49FBAAAD21
// reference: E2DC0D3B5A0161C3, FEAD9BDB63FFC15E
// reference: 2CF663CC2E9D28AC, D13EA8CA11590A72
// shader: 8B30, 3DDAD0187314463A
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (refl_value * light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - rounded_primary_color.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 650511F19FB505C9, 3DDAD0187314463A
// program: B710898567ABFB66, 1E9B8787AEA67ABF, 3DDAD0187314463A
// reference: 55D92653F24BB4D9, A480AE4BE233C30A
// reference: 537E6FBC05851C59, 2B434053B4DE3312
// shader: 8B30, 8A0437B7998E0B4F
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (refl_value * light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (secondary_fragment_color.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((secondary_fragment_color.b) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.aaa) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - rounded_primary_color.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5296F4D09FB505C9, 8A0437B7998E0B4F
// program: B710898567ABFB66, 1E9B8787AEA67ABF, 8A0437B7998E0B4F
// reference: 624AC372F24BB4D9, 6837A7620189D676
// reference: 624AC372C2F3AB92, 6837A7620189D676
// reference: E7DA7D015A0161C3, FEAD9BDB63FFC15E
// reference: E124E52BED794189, FEAD9BDB63FFC15E
// reference: FD037D3E6A12B122, FEAD9BDB63FFC15E
// reference: 675D2E9E560CF204, D13EA8CA11590A72
// reference: 1E726B018ADA6E71, A480AE4BE233C30A
// reference: 18D522EE7D14C6F1, 2B434053B4DE3312
// reference: 29E18E208ADA6E71, 6837A7620189D676
// reference: 29E18E20BA62713A, 6837A7620189D676
// shader: 8B30, 00220421F05C8F79
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (refl_value * light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[4].position + view);
spot_dir = light_src[4].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[4].diffuse * dot_product) + light_src[4].ambient) * LookupLightingLUTUnsigned(20, clamp(light_src[4].dist_atten_scale * length(-light_src[4].position - view) + light_src[4].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[4].specular_0) + (refl_value * light_src[4].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(20, clamp(light_src[4].dist_atten_scale * length(-light_src[4].position - view) + light_src[4].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = (secondary_fragment_color.b);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (texcolor1.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (const_color[5].aaa) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5FC45E24BD1022E6, 00220421F05C8F79
// program: B710898567ABFB66, 1E9B8787AEA67ABF, 00220421F05C8F79
// shader: 8B30, D792FCC078AB5A09
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (refl_value * light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-light_src[3].position - view) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = (secondary_fragment_color.b);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (texcolor1.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (const_color[5].aaa) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5AC22E1EBD1022E6, D792FCC078AB5A09
// program: B710898567ABFB66, 1E9B8787AEA67ABF, D792FCC078AB5A09
// shader: 8B30, 8C4FC8E96EBBDA15
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
layout(binding=0, r32ui) uniform uimage2D shadow_buffer;
layout(binding=1, r32ui) uniform readonly uimage2D shadow_texture_px;

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

uvec2 DecodeShadow(uint pixel) {
    return uvec2(pixel >> 8, pixel & 255u);
}
uint EncodeShadow(uvec2 pixel) {
    return (pixel.x << 8) | pixel.y;
}
float CompareShadow(uint pixel, uint z) {
    uvec2 p = DecodeShadow(pixel);
    return mix(float(p.y) * (1.0 / 255.0), 0.0, p.x <= z);
}
float SampleShadow2D(ivec2 uv, uint z) {
    if (any(bvec4(lessThan(uv, ivec2(0)), greaterThanEqual(uv, imageSize(shadow_texture_px)))))
        return 1.0;
    return CompareShadow(imageLoad(shadow_texture_px, uv).x, z);
}
float mix2(vec4 s, vec2 a) {
    vec2 t = mix(s.xy, s.zw, a.yy);
    return mix(t.x, t.y, a.x);
}
vec4 shadowTexture(vec2 uv, float w) {
    uint z = uint(max(0, int(min(abs(w), 1.0) * (exp2(24.0) - 1.0)) - shadow_texture_bias));
    vec2 coord = vec2(imageSize(shadow_texture_px)) * uv - vec2(0.5);
    vec2 coord_floor = floor(coord);
    vec2 f = coord - coord_floor;
    ivec2 i = ivec2(coord_floor);
    vec4 s = vec4(
        SampleShadow2D(i              , z),
        SampleShadow2D(i + ivec2(1, 0), z),
        SampleShadow2D(i + ivec2(0, 1), z),
        SampleShadow2D(i + ivec2(1, 1), z));
    return vec4(mix2(s, f));
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
vec4 shadow = shadowTexture(texcoord0, texcoord0_w);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0 * shadow.rgb;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0 * shadow.rgb;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(light_vector, normal), 0.0)));
refl_value.g = (lut_scale_rg * LookupLightingLUTUnsigned(5, max(dot(normal, normalize(view)), 0.0)));
refl_value.b = (lut_scale_rb * LookupLightingLUTUnsigned(4, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (refl_value * light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-light_src[2].position - view) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (secondary_fragment_color.rrr) + (last_tex_env_out.rgb) * (vec3(1) - (secondary_fragment_color.rrr)), vec3(0), vec3(1)));
float alpha_output_1 = (secondary_fragment_color.b);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (texcolor1.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (const_color[5].aaa) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5C3CB6340A6802AC, 8C4FC8E96EBBDA15
// program: B710898567ABFB66, 1E9B8787AEA67ABF, 8C4FC8E96EBBDA15

// shader: 8DD9, 64BE452CE0E53833

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
    texcoord2 = vec2(vtx.attributes[3].x, vtx.attributes[3].y);

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
// reference: 6388F1D71180E63A, 64BE452CE0E53833
// shader: 8B31, A8402D5D722D637F

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp15 = uniforms.f[92].xxxz;
reg_tmp15.xyz = (vs_in_reg0).xyz;
reg_tmp10 = vs_in_reg3;
reg_tmp10.z = (uniforms.f[92].zzzz).z;
reg_tmp11 = uniforms.f[92].zzzz;
reg_tmp11.x = dot_3(uniforms.f[16].xyz, reg_tmp10.xyz);
reg_tmp11.y = dot_3(uniforms.f[17].xyz, reg_tmp10.xyz);
vs_out_attr0.x = dot_s(uniforms.f[12], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[13], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[14], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[15], reg_tmp15);
vs_out_attr1 = vs_in_reg1;
vs_out_attr2 = vs_in_reg2;
vs_out_attr3 = reg_tmp11;
return true;
}
// reference: A0CFFB5C19A81EC8, A8402D5D722D637F
// shader: 8B30, 4FF2B60CA3F55D65
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].aaa), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((texcolor0.g) * (const_color[1].g) + (last_tex_env_out.g), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.b) * (const_color[2].b) + (last_tex_env_out.b), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5 * 1.0, alpha_output_5 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B30B83DBABD75C2B, 4FF2B60CA3F55D65
// program: A8402D5D722D637F, 64BE452CE0E53833, 4FF2B60CA3F55D65
// shader: 8B30, 70CF1DF725C8A199
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
// reference: 003107585932B20C, 70CF1DF725C8A199
// program: A8402D5D722D637F, 64BE452CE0E53833, 70CF1DF725C8A199
// shader: 8B30, 3195BF5BA5FA14CE
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
// reference: 003107587B023B64, 3195BF5BA5FA14CE
// program: A8402D5D722D637F, 64BE452CE0E53833, 3195BF5BA5FA14CE
// shader: 8B30, 081F4E6049044080
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rrr), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rrr) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor2.rgb) * (const_color[2].rrr) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 97F2D3F6FD4C1718, 081F4E6049044080
// program: A8402D5D722D637F, 64BE452CE0E53833, 081F4E6049044080
// shader: 8B30, 63155AFB1568E4DB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rrr), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rrr) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rrr), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rrr), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4B264B7D69F882A3, 63155AFB1568E4DB
// program: A8402D5D722D637F, 64BE452CE0E53833, 63155AFB1568E4DB
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
// shader: 8B31, BF0C049D57B9F179

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

bool sub_1();
bool sub_2();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp1.z = (uniforms.f[5].xxxx + reg_tmp15.zzzz).z;
sub_2();
reg_tmp0.xy = (mul_s(reg_tmp1.xyyy, reg_tmp1.zzzz)).xy;
reg_tmp0.xy = (mul_s(uniforms.f[5].yzzz, reg_tmp0.xyyy)).xy;
reg_tmp15.xy = (uniforms.f[7].xyyy + reg_tmp0.xyyy).xy;
reg_tmp15.z = (uniforms.f[7].zzzz + -reg_tmp1.zzzz).z;
return false;
}
bool sub_2() {
reg_tmp0.x = (-uniforms.f[6].xxxx + reg_tmp15.xxxx).x;
reg_tmp0.y = (uniforms.f[6].zzzz + -reg_tmp15.yyyy).y;
reg_tmp0.xy = (mul_s(uniforms.f[6].ywww, reg_tmp0.xyyy)).xy;
reg_tmp1.xy = (reg_tmp0.xyyy).xy;
return false;
}
bool sub_0() {
reg_tmp15 = uniforms.f[92].xxxz;
reg_tmp15.xyz = (vs_in_reg0.xyzz).xyz;
sub_1();
vs_out_attr0.x = dot_s(uniforms.f[8], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[9], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[10], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[11], reg_tmp15);
vs_out_attr1 = mul_s(uniforms.f[91], vs_in_reg1);
return true;
}
// reference: 994C9B7FDD7FE583, BF0C049D57B9F179
// shader: 8B30, 920194F571A1C470
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
// reference: 37A2E27937A0064F, 920194F571A1C470
// program: BF0C049D57B9F179, 422B94F1339C42ED, 920194F571A1C470
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
// shader: 8B31, 5E172D0FA1530810

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
bool sub_2();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp1.z = (uniforms.f[5].xxxx + reg_tmp15.zzzz).z;
sub_2();
reg_tmp0.xy = (mul_s(reg_tmp1.xyyy, reg_tmp1.zzzz)).xy;
reg_tmp0.xy = (mul_s(uniforms.f[5].yzzz, reg_tmp0.xyyy)).xy;
reg_tmp15.xy = (uniforms.f[7].xyyy + reg_tmp0.xyyy).xy;
reg_tmp15.z = (uniforms.f[7].zzzz + -reg_tmp1.zzzz).z;
return false;
}
bool sub_2() {
reg_tmp0.x = (-uniforms.f[6].xxxx + reg_tmp15.xxxx).x;
reg_tmp0.y = (uniforms.f[6].zzzz + -reg_tmp15.yyyy).y;
reg_tmp0.xy = (mul_s(uniforms.f[6].ywww, reg_tmp0.xyyy)).xy;
reg_tmp1.xy = (reg_tmp0.xyyy).xy;
return false;
}
bool sub_0() {
reg_tmp15 = uniforms.f[92].xxxz;
reg_tmp15.xyz = (vs_in_reg0.xyzz).xyz;
sub_1();
reg_tmp5.x = dot_s(uniforms.f[8], reg_tmp15);
reg_tmp5.y = dot_s(uniforms.f[9], reg_tmp15);
reg_tmp5.z = dot_s(uniforms.f[10], reg_tmp15);
reg_tmp5.w = dot_s(uniforms.f[11], reg_tmp15);
vs_out_attr0 = reg_tmp5;
vs_out_attr2 = mul_s(uniforms.f[91], vs_in_reg2);
vs_out_attr1 = vs_in_reg1.xyyy;
return true;
}
// reference: 36080938D8FE2F3B, 5E172D0FA1530810
// shader: 8B30, 53EFD3479A784265
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3F2FBEA7C2A8FE13, 53EFD3479A784265
// program: 5E172D0FA1530810, AF3D45D1FD56EDEB, 53EFD3479A784265
// shader: 8B30, A1AAA0464E3057D3
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6E8591CAC2A8FE13, A1AAA0464E3057D3
// program: 5E172D0FA1530810, AF3D45D1FD56EDEB, A1AAA0464E3057D3
// shader: 8B30, F3A5AD52D9818EEF
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7F9FFA27A0A1673, F3A5AD52D9818EEF
// program: 5E172D0FA1530810, AF3D45D1FD56EDEB, F3A5AD52D9818EEF
// shader: 8B30, E26DD8B9DA6EA50B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 65340EFA7A0A1673, E26DD8B9DA6EA50B
// program: 5E172D0FA1530810, AF3D45D1FD56EDEB, E26DD8B9DA6EA50B
// shader: 8B30, A57E2E42DA6EA50B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
// reference: 030DC4B67A0A1673, A57E2E42DA6EA50B
// program: 5E172D0FA1530810, AF3D45D1FD56EDEB, A57E2E42DA6EA50B
// shader: 8B30, 22E0B91DDCF28ED5
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6BB16173504995A5, 22E0B91DDCF28ED5
// program: 5E172D0FA1530810, AF3D45D1FD56EDEB, 22E0B91DDCF28ED5
// shader: 8B30, A161E5657F514304
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].aaa), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((texcolor0.g) * (const_color[1].g) + (last_tex_env_out.g), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.b) * (const_color[2].b) + (last_tex_env_out.b), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5 * 1.0, alpha_output_5 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B30B83DB6D219E3F, A161E5657F514304
// program: A8402D5D722D637F, 64BE452CE0E53833, A161E5657F514304
// shader: 8B30, 70FFC45C4844A7AB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 003107589FC47018, 70FFC45C4844A7AB
// program: A8402D5D722D637F, 64BE452CE0E53833, 70FFC45C4844A7AB
// shader: 8B30, C57CD65AC6CD92A4
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 00310758BDF4F970, C57CD65AC6CD92A4
// program: A8402D5D722D637F, 64BE452CE0E53833, C57CD65AC6CD92A4
// shader: 8B30, 05C754A9E550E5AD
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rrr), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rrr) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor2.rgb) * (const_color[2].rrr) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 97F2D3F63BBAD50C, 05C754A9E550E5AD
// program: A8402D5D722D637F, 64BE452CE0E53833, 05C754A9E550E5AD
// shader: 8B30, D940E6530D7C5E76
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rrr), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rrr) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rrr), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rrr), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4B264B7DAF0E40B7, D940E6530D7C5E76
// program: A8402D5D722D637F, 64BE452CE0E53833, D940E6530D7C5E76
// shader: 8B30, 3A96D8229A784265
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 591674EBC2A8FE13, 3A96D8229A784265
// program: 5E172D0FA1530810, AF3D45D1FD56EDEB, 3A96D8229A784265
// shader: 8B31, E1376E530AE24D2A

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

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp15 = uniforms.f[92].xxxz;
reg_tmp15.xyz = (vs_in_reg0.xyzz).xyz;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
vs_out_attr2 = mul_s(uniforms.f[91], vs_in_reg2);
vs_out_attr1 = vs_in_reg1.xyyy;
return true;
}
// reference: 36080938FC986F1A, E1376E530AE24D2A
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, 22E0B91DDCF28ED5
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, A1AAA0464E3057D3
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, F3A5AD52D9818EEF
// shader: 8DD9, F3E3101E81A13CCB

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
    return vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
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

    vec4 vtx_color = vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z);
    texcoord2 = vec2(vtx.attributes[6].x, vtx.attributes[6].y);

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
// reference: 93CC74B3CA1C8C74, F3E3101E81A13CCB
// shader: 8B31, A647F14B0179FC4B

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
bool sub_4();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp6.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp6.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp6.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12.x = dot_3(uniforms.f[25].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27].xyz, reg_tmp14.xyz);
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_0() {
sub_1();
sub_2();
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp8.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp8.zw = (uniforms.f[93].xxyy).zw;
reg_tmp9 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp9.xyzz)).xyz;
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp8);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp8);
reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
vs_out_attr3 = reg_tmp9;
vs_out_attr4 = reg_tmp3.xyyy;
vs_out_attr5 = reg_tmp4;
vs_out_attr6 = reg_tmp5;
return true;
}
// reference: 1703B5A9CE7ABA6F, A647F14B0179FC4B
// shader: 8B30, 44F33DD985C4D1DA
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (const_color[1].rgb) * (vec3(1) - (rounded_primary_color.aaa)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 28270769ADED5D72, 44F33DD985C4D1DA
// program: A647F14B0179FC4B, F3E3101E81A13CCB, 44F33DD985C4D1DA
// shader: 8DD9, B24DF5DBCE8D1417

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
    return vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
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

    vec4 vtx_color = vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(0.0, 0.0);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z);
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
// reference: A23A9E792464D9AE, B24DF5DBCE8D1417
// shader: 8B31, 22118D45A7CD8526

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
bool sub_0();
bool sub_5();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp6.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp6.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp6.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12.x = dot_3(uniforms.f[25].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27].xyz, reg_tmp14.xyz);
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_0() {
sub_1();
sub_2();
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp9 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
reg_tmp9.w = (mul_s(uniforms.f[21].wwww, reg_tmp9.wwww)).w;
reg_tmp0.y = (uniforms.f[7].wwww).y;
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
bool_regs = equal(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp9.xyzz)).xyz;
vs_out_attr4 = reg_tmp3.xyyy;
if (bool_regs.y) {
sub_5();
}
vs_out_attr3 = reg_tmp9;
return true;
}
bool sub_5() {
reg_tmp9 = uniforms.f[21];
return false;
}
// reference: 2A47F00366749963, 22118D45A7CD8526
// shader: 8B30, 11A4FA7F2696CF51
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (const_color[1].rgb) * (vec3(1) - (rounded_primary_color.aaa)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 35FE76E811C9484F, 11A4FA7F2696CF51
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 11A4FA7F2696CF51
// shader: 8B30, 68398773CBA417E8
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5A2AB705E5DDBE64, 68398773CBA417E8
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 68398773CBA417E8
// shader: 8B30, ACD5E9C5D013719B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 00310758C1214ED6, ACD5E9C5D013719B
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, ACD5E9C5D013719B
// shader: 8B30, 0C52A6064F4794C4
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E219756CD1705, 0C52A6064F4794C4
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 0C52A6064F4794C4
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, A57E2E42DA6EA50B
// shader: 8B30, 7F455826CF3F18FD
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7BC284B9735668FA, 7F455826CF3F18FD
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, 7F455826CF3F18FD
// shader: 8B30, 556D6044CF3F18FD
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1DFB4EF5735668FA, 556D6044CF3F18FD
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, 556D6044CF3F18FD
// shader: 8B30, 3A96D82231A3A0A9
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DAFC2F0AC2A8FE13, 3A96D82231A3A0A9
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, 3A96D82231A3A0A9
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, 3A96D8229A784265
// shader: 8B30, A0F69268E3FED43F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E942067EE8936DB4, A0F69268E3FED43F
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, A0F69268E3FED43F
// shader: 8B30, 19B10F2B407A3B3A
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3D1D02973777E9D7, 19B10F2B407A3B3A
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 19B10F2B407A3B3A
// shader: 8B30, EA0F5E892CD7D350
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].aaa), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B2577C70B72C8F18, EA0F5E892CD7D350
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, EA0F5E892CD7D350
// shader: 8B31, BDF7416D65A6B66F

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
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

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
bool sub_4();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp2.xy);
reg_tmp1 = mul_s(uniforms.f[8].wwww, vs_in_reg8);
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp3.x = dot_s(uniforms.f[23 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[24 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[23 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[24 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_s(uniforms.f[23 + addr_regs.y], reg_tmp15);
reg_tmp2.y = dot_s(uniforms.f[24 + addr_regs.y], reg_tmp15);
reg_tmp2.z = dot_s(uniforms.f[25 + addr_regs.y], reg_tmp15);
reg_tmp5.x = dot_3(uniforms.f[23 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.y = dot_3(uniforms.f[24 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.z = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp6 = mul_s(reg_tmp1.xxxx, reg_tmp3);
reg_tmp12 = mul_s(reg_tmp1.xxxx, reg_tmp4);
reg_tmp6 = fma_s(reg_tmp1.yyyy, reg_tmp2, reg_tmp6);
reg_tmp12 = fma_s(reg_tmp1.yyyy, reg_tmp5, reg_tmp12);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_0() {
sub_1();
vs_out_attr3 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
sub_2();
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
vs_out_attr4 = reg_tmp3.xyyy;
vs_out_attr5 = reg_tmp3.xyyy;
vs_out_attr6 = reg_tmp3.xyyy;
return true;
}
// reference: 1703B5A9AE747EBB, BDF7416D65A6B66F
// shader: 8B30, C9A51E99424C022E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.aaa) * (last_tex_env_out.rgb) + (const_color[1].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 78F5EF6A12D91AE9, C9A51E99424C022E
// program: BDF7416D65A6B66F, F3E3101E81A13CCB, C9A51E99424C022E
// shader: 8B30, 988962B22BE0BBA4
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
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
// reference: 8BF59422FC419050, 988962B22BE0BBA4
// program: BDF7416D65A6B66F, F3E3101E81A13CCB, 988962B22BE0BBA4
// shader: 8B30, D762CA09868F4430
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BC26D805564BE8ED, D762CA09868F4430
// program: BDF7416D65A6B66F, F3E3101E81A13CCB, D762CA09868F4430
// shader: 8B31, A0B8370D113E7E49

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
bool sub_4();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp6.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp6.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp6.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12.x = dot_3(uniforms.f[25].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27].xyz, reg_tmp14.xyz);
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_0() {
sub_1();
sub_2();
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp9 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp9.xyzz)).xyz;
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
vs_out_attr4 = reg_tmp3.xyyy;
vs_out_attr3 = reg_tmp9;
vs_out_attr5 = reg_tmp4.xyyy;
vs_out_attr6 = reg_tmp5.xyyy;
return true;
}
// reference: 1703B5A99836973A, A0B8370D113E7E49
// shader: 8B30, 5E6D53E60EA76308
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (texcolor0.r);
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
// reference: 52047F9E10DF3156, 5E6D53E60EA76308
// program: A0B8370D113E7E49, F3E3101E81A13CCB, 5E6D53E60EA76308
// shader: 8B30, 39A05F7E58E24E0F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) + (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((vec3(1) - rounded_primary_color.aaa) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FA8352607444AB98, 39A05F7E58E24E0F
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 39A05F7E58E24E0F
// shader: 8B30, 00D9709CDFE46AEC
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 034D6DB0AFA49ED0, 00D9709CDFE46AEC
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 00D9709CDFE46AEC
// shader: 8B30, 74DD1E8C9BC0BA10
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - rounded_primary_color.aaa) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 319A5A1EFE15C574, 74DD1E8C9BC0BA10
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 74DD1E8C9BC0BA10
// shader: 8B30, D0EE9BE11CB5A0CF
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normalize(view), normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 186F11D50F309561, D0EE9BE11CB5A0CF
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, D0EE9BE11CB5A0CF
// shader: 8B30, 5A1FB06F571B5697
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.r);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D98045E2FB9E6B1F, 5A1FB06F571B5697
// program: A0B8370D113E7E49, F3E3101E81A13CCB, 5A1FB06F571B5697
// shader: 8B30, D44A325BA9C3E89C
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.r);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8A18D6BF28F35EB8, D44A325BA9C3E89C
// program: A0B8370D113E7E49, F3E3101E81A13CCB, D44A325BA9C3E89C
// reference: 186F11D58172245F, D0EE9BE11CB5A0CF
// shader: 8B30, 1DE5718F0A2B5E0A
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6DF9FB22CB431476, 1DE5718F0A2B5E0A
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 1DE5718F0A2B5E0A
// shader: 8B30, 42793BEE8A2F1BC3
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 034D6DB016BA69F1, 42793BEE8A2F1BC3
// program: A0B8370D113E7E49, F3E3101E81A13CCB, 42793BEE8A2F1BC3
// shader: 8B30, B0124E5D2AA57AD1
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1E941C312876FF43, B0124E5D2AA57AD1
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, B0124E5D2AA57AD1
// shader: 8B31, 85E327F85CBA222D

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
layout(location=4) in vec4 vs_in_reg4;
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

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
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp2.xy);
reg_tmp1 = mul_s(uniforms.f[8].wwww, vs_in_reg8);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
bool_regs = equal(uniforms.f[9].xy, reg_tmp15.ww);
reg_tmp3.x = dot_s(uniforms.f[23 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[24 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[23 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[24 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_s(uniforms.f[23 + addr_regs.y], reg_tmp15);
reg_tmp2.y = dot_s(uniforms.f[24 + addr_regs.y], reg_tmp15);
reg_tmp2.z = dot_s(uniforms.f[25 + addr_regs.y], reg_tmp15);
reg_tmp5.x = dot_3(uniforms.f[23 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.y = dot_3(uniforms.f[24 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.z = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp6 = mul_s(reg_tmp1.xxxx, reg_tmp3);
reg_tmp12 = mul_s(reg_tmp1.xxxx, reg_tmp4);
reg_tmp7.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.xyz = (fma_s(reg_tmp1.yyyy, reg_tmp2.xyzz, reg_tmp6.xyzz)).xyz;
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12 = fma_s(reg_tmp1.yyyy, reg_tmp5, reg_tmp12);
vs_out_attr5 = reg_tmp7.xyyy;
reg_tmp6.xyz = (uniforms.f[95] + reg_tmp6.xyzz).xyz;
reg_tmp1.x = (uniforms.f[95].wwww).x;
reg_tmp6.z = (-uniforms.f[6].wwww + reg_tmp6.zzzz).z;
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
vs_out_attr3 = mul_s(uniforms.f[21], reg_tmp11);
if (bool_regs.x) {
jmp_to = 88u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 88u:
vs_out_attr4 = reg_tmp7.xyyy;
vs_out_attr6 = reg_tmp7.xyyy;
vs_out_attr1 = reg_tmp0;
return true;
default: return false;
}
}
return false;
}
bool sub_1() {
reg_tmp6.z = (fma_s(reg_tmp6.zzzz, reg_tmp1.xxxx, uniforms.f[6].wwww)).z;
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp1 = vec4(dot_3(uniforms.f[4].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[4].wwww;
vs_out_attr2 = -reg_tmp15;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp3 = uniforms.f[22];
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp2 = uniforms.f[5] + -reg_tmp3;
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_2() {
reg_tmp6.z = (fma_s(reg_tmp6.zzzz, reg_tmp1.xxxx, uniforms.f[6].wwww)).z;
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp1 = vec4(dot_3(uniforms.f[4].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[4].wwww;
vs_out_attr2 = -reg_tmp15;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp3 = uniforms.f[22];
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp2 = uniforms.f[5] + -reg_tmp3;
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
// reference: 78668F1F834EB8CC, 85E327F85CBA222D
// shader: 8B30, 34793D88BD145CC4
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) + (((lut_scale_d1 * LookupLightingLUTSigned(1, dot(light_vector, normal))) * light_src[0].specular_1) * geo_factor)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((vec3(1) - texcolor2.rgb) + (const_color[0].aaa), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (const_color[1].a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((vec3(1) - const_color[4].aaa) * (texcolor1.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa) + (const_color[5].rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E75FE2B2B4F8CC83, 34793D88BD145CC4
// program: 85E327F85CBA222D, F3E3101E81A13CCB, 34793D88BD145CC4
// shader: 8B31, D030ACD26C536E1F

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
layout(location=4) in vec4 vs_in_reg4;
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

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

bool sub_0();
bool sub_1();
bool sub_2();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
uint jmp_to = 93u;
while (true) {
switch (jmp_to) {
case 93u:
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp2.xy);
reg_tmp1 = mul_s(uniforms.f[8].wwww, vs_in_reg8);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
bool_regs = equal(uniforms.f[9].xy, reg_tmp15.ww);
reg_tmp3.x = dot_s(uniforms.f[23 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[24 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[23 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[24 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_s(uniforms.f[23 + addr_regs.y], reg_tmp15);
reg_tmp2.y = dot_s(uniforms.f[24 + addr_regs.y], reg_tmp15);
reg_tmp2.z = dot_s(uniforms.f[25 + addr_regs.y], reg_tmp15);
reg_tmp5.x = dot_3(uniforms.f[23 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.y = dot_3(uniforms.f[24 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.z = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp6 = mul_s(reg_tmp1.xxxx, reg_tmp3);
reg_tmp12 = mul_s(reg_tmp1.xxxx, reg_tmp4);
reg_tmp7.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.xyz = (fma_s(reg_tmp1.yyyy, reg_tmp2.xyzz, reg_tmp6.xyzz)).xyz;
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12 = fma_s(reg_tmp1.yyyy, reg_tmp5, reg_tmp12);
vs_out_attr6 = reg_tmp7.xyyy;
reg_tmp6.xyz = (uniforms.f[95] + reg_tmp6.xyzz).xyz;
reg_tmp1.x = (uniforms.f[95].wwww).x;
reg_tmp6.z = (-uniforms.f[6].wwww + reg_tmp6.zzzz).z;
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
vs_out_attr3 = mul_s(uniforms.f[21], reg_tmp11);
if (bool_regs.x) {
jmp_to = 181u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 181u:
vs_out_attr4 = reg_tmp7.xyyy;
vs_out_attr1 = reg_tmp0;
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr5 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
return true;
default: return false;
}
}
return false;
}
bool sub_1() {
reg_tmp6.z = (fma_s(reg_tmp6.zzzz, reg_tmp1.xxxx, uniforms.f[6].wwww)).z;
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp1 = vec4(dot_3(uniforms.f[4].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[4].wwww;
vs_out_attr2 = -reg_tmp15;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp3 = uniforms.f[22];
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp2 = uniforms.f[5] + -reg_tmp3;
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_2() {
reg_tmp6.z = (fma_s(reg_tmp6.zzzz, reg_tmp1.xxxx, uniforms.f[6].wwww)).z;
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp1 = vec4(dot_3(uniforms.f[4].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[4].wwww;
vs_out_attr2 = -reg_tmp15;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp3 = uniforms.f[22];
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp2 = uniforms.f[5] + -reg_tmp3;
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
// reference: 78668F1F1761E82F, D030ACD26C536E1F
// shader: 8B30, 4271F240369D5DD5
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) + (((lut_scale_d1 * LookupLightingLUTSigned(1, dot(light_vector, normal))) * light_src[0].specular_1) * geo_factor)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 94B16815AB99D82C, 4271F240369D5DD5
// program: D030ACD26C536E1F, F3E3101E81A13CCB, 4271F240369D5DD5
// reference: 8E26E9D77EC63F10, 34793D88BD145CC4
// shader: 8B30, A03B23B69124FC78
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EE13A0C3A185DB42, A03B23B69124FC78
// program: A0B8370D113E7E49, F3E3101E81A13CCB, A03B23B69124FC78
// shader: 8B30, D0DD53BF5310F604
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normalize(view), normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2FFCF4F40F309561, D0DD53BF5310F604
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, D0DD53BF5310F604
// reference: 00310758C1791845, 3195BF5BA5FA14CE
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 3195BF5BA5FA14CE
// shader: 8B30, 7B0B468B9678FC57
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
// reference: 003107581DACE05E, 7B0B468B9678FC57
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 7B0B468B9678FC57
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
// shader: 8B31, 2B1CAAAAAFEAC2F6

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
bool sub_1();
bool sub_2();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp7 = uniforms.f[24].wwww + vs_in_reg1;
reg_tmp15.xyz = (vs_in_reg0).xyz;
addr_regs.xy = ivec2(reg_tmp7.xx);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
reg_tmp15.xz = (mul_s(uniforms.f[25 + addr_regs.x].wwww, reg_tmp15.xzzz)).xz;
bool_regs = equal(uniforms.f[9].xy, reg_tmp10.ww);
reg_tmp15.y = (mul_s(uniforms.f[24].yyyy, reg_tmp15.yyyy)).y;
reg_tmp15.y = (uniforms.f[24].zzzz + reg_tmp15.yyyy).y;
reg_tmp10.xyz = (uniforms.f[25 + addr_regs.x].xyzz + reg_tmp15.xyzz).xyz;
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
reg_tmp7.z = (uniforms.f[95].wwww + reg_tmp7.zzzz).z;
vs_out_attr0 = reg_tmp7;
return true;
}
bool sub_1() {
reg_tmp7.x = dot_s(uniforms.f[10], reg_tmp10);
reg_tmp7.y = dot_s(uniforms.f[11], reg_tmp10);
reg_tmp7.z = dot_s(uniforms.f[12], reg_tmp10);
reg_tmp7.w = dot_s(uniforms.f[13], reg_tmp10);
return false;
}
bool sub_2() {
reg_tmp7.x = dot_s(uniforms.f[14], reg_tmp10);
reg_tmp7.y = dot_s(uniforms.f[15], reg_tmp10);
reg_tmp7.z = dot_s(uniforms.f[16], reg_tmp10);
reg_tmp7.w = dot_s(uniforms.f[17], reg_tmp10);
return false;
}
// reference: B3F02BF7DFA05FDC, 2B1CAAAAAFEAC2F6
// program: 2B1CAAAAAFEAC2F6, E957B9D48F28DB0E, 70CF1DF725C8A199
// shader: 8B31, 1A15BB28BDA164A6

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
bool sub_1();
bool sub_2();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp15.xyz = (vs_in_reg0).xyz;
bool_regs = equal(uniforms.f[9].xy, reg_tmp15.ww);
reg_tmp15.xz = (mul_s(uniforms.f[28].xxxx, reg_tmp15.xzzz)).xz;
reg_tmp15.y = (mul_s(uniforms.f[28].yyyy, reg_tmp15.yyyy)).y;
reg_tmp15.y = (uniforms.f[28].zzzz + reg_tmp15.yyyy).y;
reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
reg_tmp7.z = (uniforms.f[95].wwww + reg_tmp7.zzzz).z;
vs_out_attr0 = reg_tmp7;
return true;
}
bool sub_1() {
reg_tmp7.x = dot_s(uniforms.f[10], reg_tmp10);
reg_tmp7.y = dot_s(uniforms.f[11], reg_tmp10);
reg_tmp7.z = dot_s(uniforms.f[12], reg_tmp10);
reg_tmp7.w = dot_s(uniforms.f[13], reg_tmp10);
return false;
}
bool sub_2() {
reg_tmp7.x = dot_s(uniforms.f[14], reg_tmp10);
reg_tmp7.y = dot_s(uniforms.f[15], reg_tmp10);
reg_tmp7.z = dot_s(uniforms.f[16], reg_tmp10);
reg_tmp7.w = dot_s(uniforms.f[17], reg_tmp10);
return false;
}
// reference: B3F02BF7D4CCFF12, 1A15BB28BDA164A6
// program: 1A15BB28BDA164A6, E957B9D48F28DB0E, 70CF1DF725C8A199
// shader: 8B31, F690B9DE63154125

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
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

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
bool sub_3();
bool sub_4();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
bool_regs.x = uniforms.f[93].zzzz.x != vs_in_reg7.x;
bool_regs.y = uniforms.f[93].zzzz.y == vs_in_reg7.y;
if (bool_regs.x) {
sub_1();
} else {
sub_2();
}
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
vs_out_attr1 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
reg_tmp7.x = (uniforms.f[94].yyyy + reg_tmp7.xxxx).x;
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
reg_tmp6.xyz = (fma_s(reg_tmp7.wwww, reg_tmp12.xyzz, reg_tmp6.xyzz)).xyz;
reg_tmp0.x = (uniforms.f[95].wwww).x;
reg_tmp6.z = (-uniforms.f[19].wwww + reg_tmp6.zzzz).z;
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
reg_tmp7.xyz = (uniforms.f[18].yyzz + reg_tmp7.xyzz).xyz;
vs_out_attr0 = reg_tmp7;
return true;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp2.xy);
reg_tmp1 = mul_s(uniforms.f[8].wwww, vs_in_reg8);
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp3.x = dot_s(uniforms.f[23 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[24 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[23 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[24 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_s(uniforms.f[23 + addr_regs.y], reg_tmp15);
reg_tmp2.y = dot_s(uniforms.f[24 + addr_regs.y], reg_tmp15);
reg_tmp2.z = dot_s(uniforms.f[25 + addr_regs.y], reg_tmp15);
reg_tmp5.x = dot_3(uniforms.f[23 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.y = dot_3(uniforms.f[24 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.z = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp6 = mul_s(reg_tmp1.xxxx, reg_tmp3);
reg_tmp12 = mul_s(reg_tmp1.xxxx, reg_tmp4);
reg_tmp6 = fma_s(reg_tmp1.yyyy, reg_tmp2, reg_tmp6);
reg_tmp12 = fma_s(reg_tmp1.yyyy, reg_tmp5, reg_tmp12);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp6.xyz = (uniforms.f[95] + reg_tmp6.xyzz).xyz;
reg_tmp7.w = (uniforms.f[18].xxxx).w;
return false;
}
bool sub_2() {
reg_tmp1 = mul_s(uniforms.f[8].wwww, vs_in_reg8);
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp6.x = dot_s(uniforms.f[23], reg_tmp15);
reg_tmp6.y = dot_s(uniforms.f[24], reg_tmp15);
reg_tmp6.z = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12.x = dot_3(uniforms.f[23].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[24].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[25].xyz, reg_tmp14.xyz);
reg_tmp7.w = (uniforms.f[18].xxxx).w;
reg_tmp6.xyz = (uniforms.f[95] + reg_tmp6.xyzz).xyz;
return false;
}
bool sub_3() {
reg_tmp6.z = (fma_s(reg_tmp6.zzzz, reg_tmp0.xxxx, uniforms.f[19].wwww)).z;
reg_tmp7.x = dot_s(uniforms.f[10], reg_tmp6);
reg_tmp7.y = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp7.z = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp7.w = dot_s(uniforms.f[13], reg_tmp6);
return false;
}
bool sub_4() {
reg_tmp6.z = (fma_s(reg_tmp6.zzzz, reg_tmp0.xxxx, uniforms.f[19].wwww)).z;
reg_tmp7.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp7.y = dot_s(uniforms.f[15], reg_tmp6);
reg_tmp7.z = dot_s(uniforms.f[16], reg_tmp6);
reg_tmp7.w = dot_s(uniforms.f[17], reg_tmp6);
return false;
}
// reference: 76D17A10CE81E704, F690B9DE63154125
// shader: 8B30, 322ED4F57F56710B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa) + (const_color[1].rgb) * (vec3(1) - (const_color[1].aaa)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a) + (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 729965E561B42874, 322ED4F57F56710B
// program: F690B9DE63154125, 422B94F1339C42ED, 322ED4F57F56710B
// shader: 8B30, 8D74F3C2AB72408D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) + (((lut_scale_d1 * LookupLightingLUTSigned(1, dot(light_vector, normal))) * light_src[0].specular_1) * geo_factor)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A3228D345DCF7F2D, 8D74F3C2AB72408D
// program: D030ACD26C536E1F, F3E3101E81A13CCB, 8D74F3C2AB72408D
// shader: 8B31, 3AE5BAF463299370

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
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

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
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_0();
bool sub_8();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp2.xy);
reg_tmp1 = mul_s(uniforms.f[8].wwww, vs_in_reg8);
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp3.x = dot_s(uniforms.f[23 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[24 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[23 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[24 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_s(uniforms.f[23 + addr_regs.y], reg_tmp15);
reg_tmp2.y = dot_s(uniforms.f[24 + addr_regs.y], reg_tmp15);
reg_tmp2.z = dot_s(uniforms.f[25 + addr_regs.y], reg_tmp15);
reg_tmp5.x = dot_3(uniforms.f[23 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.y = dot_3(uniforms.f[24 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.z = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp6 = mul_s(reg_tmp1.xxxx, reg_tmp3);
reg_tmp12 = mul_s(reg_tmp1.xxxx, reg_tmp4);
reg_tmp6 = fma_s(reg_tmp1.yyyy, reg_tmp2, reg_tmp6);
reg_tmp12 = fma_s(reg_tmp1.yyyy, reg_tmp5, reg_tmp12);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_5() {
reg_tmp1 = vec4(dot_3(uniforms.f[4].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[4].wwww;
reg_tmp9 = vs_in_reg3;
reg_tmp0.y = (uniforms.f[7].wwww).y;
reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9.w = (uniforms.f[21].wwww).w;
reg_tmp3 = uniforms.f[22];
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp2 = uniforms.f[5] + -reg_tmp3;
if (bool_regs.y) {
sub_6();
}
reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
if (uniforms.b[6]) {
sub_7();
}
reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
vs_out_attr3 = reg_tmp9;
return false;
}
bool sub_6() {
reg_tmp9 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp9.xyzz)).xyz;
return false;
}
bool sub_7() {
reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
return false;
}
bool sub_0() {
sub_1();
sub_2();
sub_5();
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
vs_out_attr6 = reg_tmp6.xyyy;
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
sub_8();
vs_out_attr4 = reg_tmp3.xyyy;
vs_out_attr5 = reg_tmp6.xyyy;
return true;
}
bool sub_8() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
return false;
}
// reference: 1703B5A95CD6F6AE, 3AE5BAF463299370
// program: 3AE5BAF463299370, F3E3101E81A13CCB, 4271F240369D5DD5
// shader: 8DD9, D9AC2D854F4D2F3B

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
    texcoord1 = vec2(vtx.attributes[3].x, vtx.attributes[3].y);

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
    prim_buffer[0].attributes = vec4[4](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0]);
    prim_buffer[1].attributes = vec4[4](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1]);
    prim_buffer[2].attributes = vec4[4](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: A55C6948CCF76B42, D9AC2D854F4D2F3B
// shader: 8B31, A1BD3EE2FA292D0C

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
bool sub_7();
bool sub_2();
bool sub_5();
bool sub_6();
bool sub_0();
bool sub_3();
bool sub_4();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_7() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_2() {
reg_tmp6.x = (uniforms.f[90].xxxx).x;
reg_tmp6.y = (uniforms.f[91].xxxx).y;
reg_tmp6.z = (uniforms.f[92].xxxx).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (uniforms.f[90].yyyy).x;
reg_tmp7.y = (uniforms.f[91].yyyy).y;
reg_tmp7.z = (uniforms.f[92].yyyy).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (uniforms.f[90].zzzz).x;
reg_tmp8.y = (uniforms.f[91].zzzz).y;
reg_tmp8.z = (uniforms.f[92].zzzz).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9.x = (uniforms.f[90].wwww).x;
reg_tmp9.y = (uniforms.f[91].wwww).y;
reg_tmp9.z = (uniforms.f[92].wwww).z;
reg_tmp9.w = (uniforms.f[0].zzzz).w;
return false;
}
bool sub_5() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_6() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
sub_2();
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(uniforms.f[0].xxxz, reg_tmp10);
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[85].xyzz + -reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (uniforms.f[5 + addr_regs.x].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_5();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_6();
sub_7();
return true;
}
bool sub_3() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_4() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: 9ECB4E96DC9B34B3, A1BD3EE2FA292D0C
// shader: 8B30, D715F9B9FD91CE99
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 06BF7E5EF6B42A4A, D715F9B9FD91CE99
// program: A1BD3EE2FA292D0C, D9AC2D854F4D2F3B, D715F9B9FD91CE99
// shader: 8B31, 3AA780F8DCD4711A

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
bool sub_7();
bool sub_2();
bool sub_5();
bool sub_6();
bool sub_0();
bool sub_3();
bool sub_4();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_7() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_2() {
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.xyz = vec3(rcp_s(uniforms.f[81].y));
reg_tmp8.xyz = (fma_s(reg_tmp6.xyzz, reg_tmp7.xyzz, uniforms.f[0].yyyy)).xyz;
reg_tmp7.xyz = (floor(reg_tmp8.xyzz)).xyz;
reg_tmp6.xyz = (fma_s(reg_tmp7.xyzz, -uniforms.f[81].yyyy, reg_tmp6.xyzz)).xyz;
reg_tmp6 = min_s(uniforms.f[81].xxxx, reg_tmp6);
reg_tmp6 = max_s(-uniforms.f[81].xxxx, reg_tmp6);
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.xxxx)).y;
reg_tmp4.x = (reg_tmp1.xxxx).x;
reg_tmp5.x = (reg_tmp1.yyyy).x;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.yyyy)).y;
reg_tmp4.y = (reg_tmp1.xxxx).y;
reg_tmp5.y = (reg_tmp1.yyyy).y;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.zzzz, reg_tmp6.zzzz)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.zzzz)).y;
reg_tmp4.z = (reg_tmp1.xxxx).z;
reg_tmp5.z = (reg_tmp1.yyyy).z;
reg_tmp2.x = (mul_s(reg_tmp4.yyyy, reg_tmp5.zzzz)).x;
reg_tmp2.y = (mul_s(reg_tmp5.yyyy, reg_tmp5.zzzz)).y;
reg_tmp6.x = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).x;
reg_tmp6.y = (reg_tmp5.zzzz).y;
reg_tmp6.z = (mul_s(-reg_tmp5.yyyy, reg_tmp4.zzzz)).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (mul_s(-reg_tmp2.xxxx, reg_tmp4.xxxx)).x;
reg_tmp7.x = (fma_s(reg_tmp5.yyyy, reg_tmp5.xxxx, reg_tmp7.xxxx)).x;
reg_tmp7.y = (mul_s(reg_tmp4.zzzz, reg_tmp4.xxxx)).y;
reg_tmp7.z = (mul_s(reg_tmp2.yyyy, reg_tmp4.xxxx)).z;
reg_tmp7.z = (fma_s(reg_tmp4.yyyy, reg_tmp5.xxxx, reg_tmp7.zzzz)).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (mul_s(reg_tmp2.xxxx, reg_tmp5.xxxx)).x;
reg_tmp8.x = (fma_s(reg_tmp5.yyyy, reg_tmp4.xxxx, reg_tmp8.xxxx)).x;
reg_tmp8.y = (mul_s(-reg_tmp4.zzzz, reg_tmp5.xxxx)).y;
reg_tmp8.z = (mul_s(-reg_tmp2.yyyy, reg_tmp5.xxxx)).z;
reg_tmp8.z = (fma_s(reg_tmp4.yyyy, reg_tmp4.xxxx, reg_tmp8.zzzz)).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
return false;
}
bool sub_5() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_6() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
reg_tmp2 = reg_tmp10.xzyw;
reg_tmp2.z = (-reg_tmp2.zzzz).z;
reg_tmp14 = reg_tmp2;
reg_tmp0 = uniforms.f[7 + addr_regs.x];
sub_2();
reg_tmp2.x = dot_s(reg_tmp14, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp14, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp14, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp14, reg_tmp9);
reg_tmp6 = uniforms.f[8 + addr_regs.x];
reg_tmp7 = uniforms.f[9 + addr_regs.x];
reg_tmp8 = uniforms.f[10 + addr_regs.x];
reg_tmp9 = uniforms.f[0].xxxz;
reg_tmp6.w = (uniforms.f[5 + addr_regs.x].xxxx).w;
reg_tmp7.w = (uniforms.f[5 + addr_regs.x].yyyy).w;
reg_tmp8.w = (uniforms.f[5 + addr_regs.x].zzzz).w;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[85].xyzz + -reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_5();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_6();
sub_7();
return true;
}
bool sub_3() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_4() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: CA04CBBD2C9EC5C2, 3AA780F8DCD4711A
// program: 3AA780F8DCD4711A, D9AC2D854F4D2F3B, D715F9B9FD91CE99
// shader: 8B31, C370F5A1D7E96057

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
bool sub_11();
bool sub_2();
bool sub_9();
bool sub_10();
bool sub_0();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_11() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_2() {
reg_tmp1.xyz = (uniforms.f[92].xyzz).xyz;
reg_tmp1.w = (uniforms.f[0].xxxx).w;
return false;
}
bool sub_9() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_10() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
sub_2();
reg_tmp2.xyz = (reg_tmp1.xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[6 + addr_regs.x].xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp4.xyz = (mul_s(reg_tmp3.yzxx, reg_tmp2.zxyy)).xyz;
reg_tmp4.xyz = (fma_s(-reg_tmp2.yzxx, reg_tmp3.zxyy, reg_tmp4)).xyz;
reg_tmp5.x = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp5.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp5.y;
if (bool_regs.x) {
sub_5();
} else {
sub_6();
}
reg_tmp4.xyz = (mul_s(reg_tmp4.xyzz, reg_tmp5.xxxx)).xyz;
reg_tmp5.xyz = (mul_s(reg_tmp3.yzxx, reg_tmp4.zxyy)).xyz;
reg_tmp5.xyz = (fma_s(-reg_tmp4.yzxx, reg_tmp3.zxyy, reg_tmp5)).xyz;
reg_tmp6.x = (reg_tmp4.xxxx).x;
reg_tmp6.y = (reg_tmp3.xxxx).y;
reg_tmp6.z = (reg_tmp5.xxxx).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (reg_tmp4.yyyy).x;
reg_tmp7.y = (reg_tmp3.yyyy).y;
reg_tmp7.z = (reg_tmp5.yyyy).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (reg_tmp4.zzzz).x;
reg_tmp8.y = (reg_tmp3.zzzz).y;
reg_tmp8.z = (reg_tmp5.zzzz).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp10, reg_tmp9);
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (-uniforms.f[85].xyzz + reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_7();
} else {
sub_8();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (uniforms.f[5 + addr_regs.x].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_9();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_10();
sub_11();
return true;
}
bool sub_3() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_4() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
bool sub_5() {
reg_tmp5.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_6() {
reg_tmp5.x = rsq_s(reg_tmp5.x);
return false;
}
bool sub_7() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_8() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: F404A5D3F17E9927, C370F5A1D7E96057
// program: C370F5A1D7E96057, D9AC2D854F4D2F3B, D715F9B9FD91CE99
// shader: 8B31, E4F05D73D0626FC6

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
bool sub_7();
bool sub_2();
bool sub_5();
bool sub_6();
bool sub_0();
bool sub_3();
bool sub_4();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_7() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_2() {
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.xyz = vec3(rcp_s(uniforms.f[81].y));
reg_tmp8.xyz = (fma_s(reg_tmp6.xyzz, reg_tmp7.xyzz, uniforms.f[0].yyyy)).xyz;
reg_tmp7.xyz = (floor(reg_tmp8.xyzz)).xyz;
reg_tmp6.xyz = (fma_s(reg_tmp7.xyzz, -uniforms.f[81].yyyy, reg_tmp6.xyzz)).xyz;
reg_tmp6 = min_s(uniforms.f[81].xxxx, reg_tmp6);
reg_tmp6 = max_s(-uniforms.f[81].xxxx, reg_tmp6);
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.xxxx)).y;
reg_tmp4.x = (reg_tmp1.xxxx).x;
reg_tmp5.x = (reg_tmp1.yyyy).x;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.yyyy)).y;
reg_tmp4.y = (reg_tmp1.xxxx).y;
reg_tmp5.y = (reg_tmp1.yyyy).y;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.zzzz, reg_tmp6.zzzz)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.zzzz)).y;
reg_tmp4.z = (reg_tmp1.xxxx).z;
reg_tmp5.z = (reg_tmp1.yyyy).z;
reg_tmp2.x = (mul_s(reg_tmp4.yyyy, reg_tmp5.zzzz)).x;
reg_tmp2.y = (mul_s(reg_tmp5.yyyy, reg_tmp5.zzzz)).y;
reg_tmp6.x = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).x;
reg_tmp6.y = (reg_tmp5.zzzz).y;
reg_tmp6.z = (mul_s(-reg_tmp5.yyyy, reg_tmp4.zzzz)).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (mul_s(-reg_tmp2.xxxx, reg_tmp4.xxxx)).x;
reg_tmp7.x = (fma_s(reg_tmp5.yyyy, reg_tmp5.xxxx, reg_tmp7.xxxx)).x;
reg_tmp7.y = (mul_s(reg_tmp4.zzzz, reg_tmp4.xxxx)).y;
reg_tmp7.z = (mul_s(reg_tmp2.yyyy, reg_tmp4.xxxx)).z;
reg_tmp7.z = (fma_s(reg_tmp4.yyyy, reg_tmp5.xxxx, reg_tmp7.zzzz)).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (mul_s(reg_tmp2.xxxx, reg_tmp5.xxxx)).x;
reg_tmp8.x = (fma_s(reg_tmp5.yyyy, reg_tmp4.xxxx, reg_tmp8.xxxx)).x;
reg_tmp8.y = (mul_s(-reg_tmp4.zzzz, reg_tmp5.xxxx)).y;
reg_tmp8.z = (mul_s(-reg_tmp2.yyyy, reg_tmp5.xxxx)).z;
reg_tmp8.z = (fma_s(reg_tmp4.yyyy, reg_tmp4.xxxx, reg_tmp8.zzzz)).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
return false;
}
bool sub_5() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_6() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
reg_tmp14 = reg_tmp10;
reg_tmp0 = uniforms.f[7 + addr_regs.x];
sub_2();
reg_tmp10.x = dot_s(reg_tmp14, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp14, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp14, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp14, reg_tmp9);
reg_tmp6 = uniforms.f[8 + addr_regs.x];
reg_tmp7 = uniforms.f[9 + addr_regs.x];
reg_tmp8 = uniforms.f[10 + addr_regs.x];
reg_tmp9 = uniforms.f[0].xxxz;
reg_tmp6.w = (uniforms.f[5 + addr_regs.x].xxxx).w;
reg_tmp7.w = (uniforms.f[5 + addr_regs.x].yyyy).w;
reg_tmp8.w = (uniforms.f[5 + addr_regs.x].zzzz).w;
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp10, reg_tmp9);
reg_tmp10 = reg_tmp2;
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[85].xyzz + -reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_5();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_6();
sub_7();
return true;
}
bool sub_3() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_4() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: 3E54A98EE706D9D8, E4F05D73D0626FC6
// program: E4F05D73D0626FC6, D9AC2D854F4D2F3B, D715F9B9FD91CE99
// shader: 8B31, B2C9336D361C5C04

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
bool sub_7();
bool sub_2();
bool sub_5();
bool sub_6();
bool sub_0();
bool sub_3();
bool sub_4();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_7() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_2() {
reg_tmp2 = uniforms.f[85];
reg_tmp2.xz = (-uniforms.f[5 + addr_regs.x].xzzz + reg_tmp2.xzzz).xz;
reg_tmp2.yw = (uniforms.f[0].xxxx).yw;
reg_tmp3.x = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp3.x = rsq_s(reg_tmp3.x);
reg_tmp2.xyz = (mul_s(reg_tmp2.xyzz, reg_tmp3.xxxx)).xyz;
reg_tmp4 = uniforms.f[0].xzxx;
reg_tmp5.xyz = (mul_s(reg_tmp2.yzxx, reg_tmp4.zxyy)).xyz;
reg_tmp5.xyz = (fma_s(-reg_tmp4.yzxx, reg_tmp2.zxyy, reg_tmp5)).xyz;
reg_tmp3.x = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp3.x = rsq_s(reg_tmp3.x);
reg_tmp5.xyz = (mul_s(reg_tmp5.xyzz, reg_tmp3.xxxx)).xyz;
reg_tmp6.x = (-reg_tmp5.xxxx).x;
reg_tmp6.y = (-reg_tmp5.yyyy).y;
reg_tmp6.z = (-reg_tmp5.zzzz).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (reg_tmp4.xxxx).x;
reg_tmp7.y = (reg_tmp4.yyyy).y;
reg_tmp7.z = (reg_tmp4.zzzz).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (-reg_tmp2.xxxx).x;
reg_tmp8.y = (-reg_tmp2.yyyy).y;
reg_tmp8.z = (-reg_tmp2.zzzz).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9.x = (uniforms.f[90].wwww).x;
reg_tmp9.y = (uniforms.f[91].wwww).y;
reg_tmp9.z = (uniforms.f[92].wwww).z;
reg_tmp9.w = (uniforms.f[0].zzzz).w;
return false;
}
bool sub_5() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_6() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
sub_2();
reg_tmp14.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp14.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp14.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp14.w = dot_s(uniforms.f[0].xxxz, reg_tmp10);
reg_tmp2 = reg_tmp14;
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[85].xyzz + -reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (uniforms.f[5 + addr_regs.x].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_5();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_6();
sub_7();
return true;
}
bool sub_3() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_4() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: 9ECB4E969D4FCB42, B2C9336D361C5C04
// program: B2C9336D361C5C04, D9AC2D854F4D2F3B, D715F9B9FD91CE99
// shader: 8B30, 12BA869543DAC901
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DD9A56FFD2FF6E0F, 12BA869543DAC901
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 12BA869543DAC901
// shader: 8B30, 94A9CEB53BFBA601
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4B1408BEC04BA8A7, 94A9CEB53BFBA601
// program: A1BD3EE2FA292D0C, D9AC2D854F4D2F3B, 94A9CEB53BFBA601
// program: B2C9336D361C5C04, D9AC2D854F4D2F3B, 94A9CEB53BFBA601
// program: C370F5A1D7E96057, D9AC2D854F4D2F3B, 94A9CEB53BFBA601
// program: E4F05D73D0626FC6, D9AC2D854F4D2F3B, 94A9CEB53BFBA601
// shader: 8B31, 2D8E060A79F7CD57

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
bool sub_8();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_0();
bool sub_4();
bool sub_5();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_8() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_2() {
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.xyz = vec3(rcp_s(uniforms.f[81].y));
reg_tmp8.xyz = (fma_s(reg_tmp6.xyzz, reg_tmp7.xyzz, uniforms.f[0].yyyy)).xyz;
reg_tmp7.xyz = (floor(reg_tmp8.xyzz)).xyz;
reg_tmp6.xyz = (fma_s(reg_tmp7.xyzz, -uniforms.f[81].yyyy, reg_tmp6.xyzz)).xyz;
reg_tmp6 = min_s(uniforms.f[81].xxxx, reg_tmp6);
reg_tmp6 = max_s(-uniforms.f[81].xxxx, reg_tmp6);
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.xxxx)).y;
reg_tmp4.x = (reg_tmp1.xxxx).x;
reg_tmp5.x = (reg_tmp1.yyyy).x;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.yyyy)).y;
reg_tmp4.y = (reg_tmp1.xxxx).y;
reg_tmp5.y = (reg_tmp1.yyyy).y;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.zzzz, reg_tmp6.zzzz)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.zzzz)).y;
reg_tmp4.z = (reg_tmp1.xxxx).z;
reg_tmp5.z = (reg_tmp1.yyyy).z;
reg_tmp2.x = (mul_s(reg_tmp4.yyyy, reg_tmp5.zzzz)).x;
reg_tmp2.y = (mul_s(reg_tmp5.yyyy, reg_tmp5.zzzz)).y;
reg_tmp6.x = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).x;
reg_tmp6.y = (reg_tmp5.zzzz).y;
reg_tmp6.z = (mul_s(-reg_tmp5.yyyy, reg_tmp4.zzzz)).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (mul_s(-reg_tmp2.xxxx, reg_tmp4.xxxx)).x;
reg_tmp7.x = (fma_s(reg_tmp5.yyyy, reg_tmp5.xxxx, reg_tmp7.xxxx)).x;
reg_tmp7.y = (mul_s(reg_tmp4.zzzz, reg_tmp4.xxxx)).y;
reg_tmp7.z = (mul_s(reg_tmp2.yyyy, reg_tmp4.xxxx)).z;
reg_tmp7.z = (fma_s(reg_tmp4.yyyy, reg_tmp5.xxxx, reg_tmp7.zzzz)).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (mul_s(reg_tmp2.xxxx, reg_tmp5.xxxx)).x;
reg_tmp8.x = (fma_s(reg_tmp5.yyyy, reg_tmp4.xxxx, reg_tmp8.xxxx)).x;
reg_tmp8.y = (mul_s(-reg_tmp4.zzzz, reg_tmp5.xxxx)).y;
reg_tmp8.z = (mul_s(-reg_tmp2.yyyy, reg_tmp5.xxxx)).z;
reg_tmp8.z = (fma_s(reg_tmp4.yyyy, reg_tmp4.xxxx, reg_tmp8.zzzz)).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
return false;
}
bool sub_3() {
reg_tmp6.x = (uniforms.f[90].xxxx).x;
reg_tmp6.y = (uniforms.f[91].xxxx).y;
reg_tmp6.z = (uniforms.f[92].xxxx).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (uniforms.f[90].yyyy).x;
reg_tmp7.y = (uniforms.f[91].yyyy).y;
reg_tmp7.z = (uniforms.f[92].yyyy).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (uniforms.f[90].zzzz).x;
reg_tmp8.y = (uniforms.f[91].zzzz).y;
reg_tmp8.z = (uniforms.f[92].zzzz).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9.x = (uniforms.f[90].wwww).x;
reg_tmp9.y = (uniforms.f[91].wwww).y;
reg_tmp9.z = (uniforms.f[92].wwww).z;
reg_tmp9.w = (uniforms.f[0].zzzz).w;
return false;
}
bool sub_6() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_7() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
reg_tmp0 = uniforms.f[7 + addr_regs.x];
sub_2();
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp10, reg_tmp9);
reg_tmp10 = reg_tmp2;
sub_3();
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(uniforms.f[0].xxxz, reg_tmp10);
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[85].xyzz + -reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_4();
} else {
sub_5();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (uniforms.f[5 + addr_regs.x].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_6();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_7();
sub_8();
return true;
}
bool sub_4() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_5() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: 4420DA630B44F628, 2D8E060A79F7CD57
// shader: 8B30, 8F3D8BBC3BFBA601
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C8FE535FC04BA8A7, 8F3D8BBC3BFBA601
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, 8F3D8BBC3BFBA601
// shader: 8B30, 0994432D43DAC901
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8E02C5A2D2FF6E0F, 0994432D43DAC901
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 0994432D43DAC901
// program: 3AA780F8DCD4711A, D9AC2D854F4D2F3B, 94A9CEB53BFBA601
// shader: 8B30, D715F9B9E6058B90
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 855525BFF6B42A4A, D715F9B9E6058B90
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, D715F9B9E6058B90
// shader: 8B30, 78DBD4DE92E533AA
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 06BF7E5EA096B218, 78DBD4DE92E533AA
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, 78DBD4DE92E533AA
// shader: 8B30, 08C8817FDCF28ED5
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0D88AB3F504995A5, 08C8817FDCF28ED5
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, 08C8817FDCF28ED5
// shader: 8B30, 2C8F67273BCF52D6
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8E02C5A270CDD6FB, 2C8F67273BCF52D6
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 2C8F67273BCF52D6
// shader: 8B30, 438F449963BC8B0F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 969832D419C91788, 438F449963BC8B0F
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 438F449963BC8B0F
// reference: 969832D46E3871B9, 438F449963BC8B0F
// shader: 8B30, BF94608A1B014688
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
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
// reference: E54102B0116333E7, BF94608A1B014688
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, BF94608A1B014688
// shader: 8B30, 1CE43E01EAE9F245
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (const_color[1].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ABB4371B0C76AB9E, 1CE43E01EAE9F245
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 1CE43E01EAE9F245
// shader: 8B30, 4A60600E5756925D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E2197C8376808, 4A60600E5756925D
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 4A60600E5756925D
// shader: 8B30, 50CA766563BC8B0F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C500A18968609FA4, 50CA766563BC8B0F
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 50CA766563BC8B0F
// reference: C500A18919C91788, 50CA766563BC8B0F
// shader: 8B30, 92EAFB309BF64569
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (last_tex_env_out.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((rounded_primary_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E886C43CE54F177A, 92EAFB309BF64569
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 92EAFB309BF64569
// shader: 8B30, 869CB2720D7DC9AB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (last_tex_env_out.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9B5FF458E54F177A, 869CB2720D7DC9AB
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 869CB2720D7DC9AB
// shader: 8B30, B177E2FF183ACC19
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ED6FCA2B09A8F8EA, B177E2FF183ACC19
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, B177E2FF183ACC19
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, 94A9CEB53BFBA601
// reference: FDC86370AB99D82C, 4271F240369D5DD5
// shader: 8B30, 250022C00198DA3A
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 48A2735D0DF3DBFD, 250022C00198DA3A
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 250022C00198DA3A
// shader: 8B30, 405ED72007C864DE
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3D1D0297BFC60E39, 405ED72007C864DE
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 405ED72007C864DE
// reference: 3D1D0297C8376808, 405ED72007C864DE
// reference: 349E2197BFC60E39, 4A60600E5756925D
// reference: 3D1D0297D2FF6E0F, 405ED72007C864DE
// shader: 8B30, 8AA05563B09136D8
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8E02C5A2BC4BF89D, 8AA05563B09136D8
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 8AA05563B09136D8
// shader: 8B30, CA8A7A5780C75620
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 09B22458B6887E74, CA8A7A5780C75620
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, CA8A7A5780C75620
// shader: 8B30, 27828F0B42C318F3
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DD9A56FF88E4DE52, 27828F0B42C318F3
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 27828F0B42C318F3
// shader: 8B30, 847FA0920ED4A210
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (last_tex_env_out.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((rounded_primary_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E886C43C913387EF, 847FA0920ED4A210
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 847FA0920ED4A210
// shader: 8B30, BD5062D11F60B292
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (last_tex_env_out.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9B5FF4588C2DBE56, BD5062D11F60B292
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, BD5062D11F60B292
// reference: 9B5FF4582BF5F319, 869CB2720D7DC9AB
// shader: 8B30, D333CDAFEF553CCD
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) + (((lut_scale_d1 * LookupLightingLUTSigned(1, dot(light_vector, normal))) * light_src[0].specular_1) * geo_factor)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3 * 2.0, alpha_output_3 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa) + (const_color[5].rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 41D0E532E891F47E, D333CDAFEF553CCD
// program: 3AE5BAF463299370, F3E3101E81A13CCB, D333CDAFEF553CCD
// shader: 8B30, 3EF703BBC60081BA
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (vec3(1) - texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5616CA3E25804D9C, 3EF703BBC60081BA
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 3EF703BBC60081BA
// shader: 8B30, 43863E198C1361D6
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3D1D02973BF47733, 43863E198C1361D6
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 43863E198C1361D6
// shader: 8B30, C67708DBC60081BA
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (vec3(1) - texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 058E596325804D9C, C67708DBC60081BA
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, C67708DBC60081BA
// shader: 8B30, 37A1A29F3BCF52D6
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DD9A56FF70CDD6FB, 37A1A29F3BCF52D6
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 37A1A29F3BCF52D6
// reference: E886C43CFF87117D, 92EAFB309BF64569
// shader: 8B31, 211B01F5C3FF770B

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
bool sub_12();
bool sub_2();
bool sub_3();
bool sub_10();
bool sub_11();
bool sub_0();
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

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_12() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_2() {
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.xyz = vec3(rcp_s(uniforms.f[81].y));
reg_tmp8.xyz = (fma_s(reg_tmp6.xyzz, reg_tmp7.xyzz, uniforms.f[0].yyyy)).xyz;
reg_tmp7.xyz = (floor(reg_tmp8.xyzz)).xyz;
reg_tmp6.xyz = (fma_s(reg_tmp7.xyzz, -uniforms.f[81].yyyy, reg_tmp6.xyzz)).xyz;
reg_tmp6 = min_s(uniforms.f[81].xxxx, reg_tmp6);
reg_tmp6 = max_s(-uniforms.f[81].xxxx, reg_tmp6);
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.xxxx)).y;
reg_tmp4.x = (reg_tmp1.xxxx).x;
reg_tmp5.x = (reg_tmp1.yyyy).x;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.yyyy)).y;
reg_tmp4.y = (reg_tmp1.xxxx).y;
reg_tmp5.y = (reg_tmp1.yyyy).y;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.zzzz, reg_tmp6.zzzz)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.zzzz)).y;
reg_tmp4.z = (reg_tmp1.xxxx).z;
reg_tmp5.z = (reg_tmp1.yyyy).z;
reg_tmp2.x = (mul_s(reg_tmp4.yyyy, reg_tmp5.zzzz)).x;
reg_tmp2.y = (mul_s(reg_tmp5.yyyy, reg_tmp5.zzzz)).y;
reg_tmp6.x = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).x;
reg_tmp6.y = (reg_tmp5.zzzz).y;
reg_tmp6.z = (mul_s(-reg_tmp5.yyyy, reg_tmp4.zzzz)).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (mul_s(-reg_tmp2.xxxx, reg_tmp4.xxxx)).x;
reg_tmp7.x = (fma_s(reg_tmp5.yyyy, reg_tmp5.xxxx, reg_tmp7.xxxx)).x;
reg_tmp7.y = (mul_s(reg_tmp4.zzzz, reg_tmp4.xxxx)).y;
reg_tmp7.z = (mul_s(reg_tmp2.yyyy, reg_tmp4.xxxx)).z;
reg_tmp7.z = (fma_s(reg_tmp4.yyyy, reg_tmp5.xxxx, reg_tmp7.zzzz)).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (mul_s(reg_tmp2.xxxx, reg_tmp5.xxxx)).x;
reg_tmp8.x = (fma_s(reg_tmp5.yyyy, reg_tmp4.xxxx, reg_tmp8.xxxx)).x;
reg_tmp8.y = (mul_s(-reg_tmp4.zzzz, reg_tmp5.xxxx)).y;
reg_tmp8.z = (mul_s(-reg_tmp2.yyyy, reg_tmp5.xxxx)).z;
reg_tmp8.z = (fma_s(reg_tmp4.yyyy, reg_tmp4.xxxx, reg_tmp8.zzzz)).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
return false;
}
bool sub_3() {
reg_tmp1.xyz = (uniforms.f[92].xyzz).xyz;
reg_tmp1.w = (uniforms.f[0].xxxx).w;
return false;
}
bool sub_10() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_11() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
reg_tmp0 = uniforms.f[7 + addr_regs.x];
sub_2();
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp10, reg_tmp9);
reg_tmp10 = reg_tmp2;
sub_3();
reg_tmp2.xyz = (reg_tmp1.xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[6 + addr_regs.x].xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_4();
} else {
sub_5();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp4.xyz = (mul_s(reg_tmp3.yzxx, reg_tmp2.zxyy)).xyz;
reg_tmp4.xyz = (fma_s(-reg_tmp2.yzxx, reg_tmp3.zxyy, reg_tmp4)).xyz;
reg_tmp5.x = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp5.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp5.y;
if (bool_regs.x) {
sub_6();
} else {
sub_7();
}
reg_tmp4.xyz = (mul_s(reg_tmp4.xyzz, reg_tmp5.xxxx)).xyz;
reg_tmp5.xyz = (mul_s(reg_tmp3.yzxx, reg_tmp4.zxyy)).xyz;
reg_tmp5.xyz = (fma_s(-reg_tmp4.yzxx, reg_tmp3.zxyy, reg_tmp5)).xyz;
reg_tmp6.x = (reg_tmp4.xxxx).x;
reg_tmp6.y = (reg_tmp3.xxxx).y;
reg_tmp6.z = (reg_tmp5.xxxx).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (reg_tmp4.yyyy).x;
reg_tmp7.y = (reg_tmp3.yyyy).y;
reg_tmp7.z = (reg_tmp5.yyyy).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (reg_tmp4.zzzz).x;
reg_tmp8.y = (reg_tmp3.zzzz).y;
reg_tmp8.z = (reg_tmp5.zzzz).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp10, reg_tmp9);
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (-uniforms.f[85].xyzz + reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_8();
} else {
sub_9();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (uniforms.f[5 + addr_regs.x].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_10();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_11();
sub_12();
return true;
}
bool sub_4() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_5() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
bool sub_6() {
reg_tmp5.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_7() {
reg_tmp5.x = rsq_s(reg_tmp5.x);
return false;
}
bool sub_8() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_9() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: F404A5D30633097A, 211B01F5C3FF770B
// shader: 8B30, 300CCE0DA51FD1E6
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
vec3 color_output_1 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 84A8D3A73E793E2B, 300CCE0DA51FD1E6
// program: 211B01F5C3FF770B, D9AC2D854F4D2F3B, 300CCE0DA51FD1E6
// shader: 8B30, B8762D381189F738
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
vec3 color_output_1 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C903A5470886BCC6, B8762D381189F738
// program: 211B01F5C3FF770B, D9AC2D854F4D2F3B, B8762D381189F738
// shader: 8B30, A5737E0E89A56514
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (vec3(1) - texcolor0.rgb) + (vec3(1) - const_color[0].rgb) * (vec3(1) - (vec3(1) - texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D2CB4889D3F3681B, A5737E0E89A56514
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, A5737E0E89A56514
// shader: 8B30, A62323BBF0AD258D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DD9A56FF8D9A81E0, A62323BBF0AD258D
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, A62323BBF0AD258D
// shader: 8B30, FBDD338694358B66
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4B1408BE966930F5, FBDD338694358B66
// program: A1BD3EE2FA292D0C, D9AC2D854F4D2F3B, FBDD338694358B66
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, FBDD338694358B66
// reference: 28A9EE5722AF07ED, D333CDAFEF553CCD
// shader: 8B30, BB8BD53EC3ECC20E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3 * 2.0, alpha_output_3 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa) + (const_color[5].rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 39EEDB8C36829F8B, BB8BD53EC3ECC20E
// program: 3AE5BAF463299370, F3E3101E81A13CCB, BB8BD53EC3ECC20E
// shader: 8B30, F4FF6E3996EDEB19
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3 * 2.0, alpha_output_3 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa) + (const_color[5].rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 39EEDB8C6C992FD6, F4FF6E3996EDEB19
// program: 3AE5BAF463299370, F3E3101E81A13CCB, F4FF6E3996EDEB19
// reference: ABB4371B69E5EFC7, 1CE43E01EAE9F245
// shader: 8B30, 468E3AC9726DE092
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6E8591CA3133B7BB, 468E3AC9726DE092
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, 468E3AC9726DE092
// shader: 8B30, 165E9866130E3B99
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 232EE72A07CC3556, 165E9866130E3B99
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, 165E9866130E3B99
// shader: 8B30, 20450603514F996E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8E4F8A12A41D1B3E, 20450603514F996E
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, 20450603514F996E
// shader: 8B31, B53793A9026DE268

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
bool sub_8();
bool sub_3();
bool sub_2();
bool sub_6();
bool sub_7();
bool sub_0();
bool sub_4();
bool sub_5();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_8() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_3() {
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.xyz = vec3(rcp_s(uniforms.f[81].y));
reg_tmp8.xyz = (fma_s(reg_tmp6.xyzz, reg_tmp7.xyzz, uniforms.f[0].yyyy)).xyz;
reg_tmp7.xyz = (floor(reg_tmp8.xyzz)).xyz;
reg_tmp6.xyz = (fma_s(reg_tmp7.xyzz, -uniforms.f[81].yyyy, reg_tmp6.xyzz)).xyz;
reg_tmp6 = min_s(uniforms.f[81].xxxx, reg_tmp6);
reg_tmp6 = max_s(-uniforms.f[81].xxxx, reg_tmp6);
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.xxxx)).y;
reg_tmp4.x = (reg_tmp1.xxxx).x;
reg_tmp5.x = (reg_tmp1.yyyy).x;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.yyyy)).y;
reg_tmp4.y = (reg_tmp1.xxxx).y;
reg_tmp5.y = (reg_tmp1.yyyy).y;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.zzzz, reg_tmp6.zzzz)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.zzzz)).y;
reg_tmp4.z = (reg_tmp1.xxxx).z;
reg_tmp5.z = (reg_tmp1.yyyy).z;
reg_tmp2.x = (mul_s(reg_tmp4.yyyy, reg_tmp5.zzzz)).x;
reg_tmp2.y = (mul_s(reg_tmp5.yyyy, reg_tmp5.zzzz)).y;
reg_tmp6.x = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).x;
reg_tmp6.y = (reg_tmp5.zzzz).y;
reg_tmp6.z = (mul_s(-reg_tmp5.yyyy, reg_tmp4.zzzz)).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (mul_s(-reg_tmp2.xxxx, reg_tmp4.xxxx)).x;
reg_tmp7.x = (fma_s(reg_tmp5.yyyy, reg_tmp5.xxxx, reg_tmp7.xxxx)).x;
reg_tmp7.y = (mul_s(reg_tmp4.zzzz, reg_tmp4.xxxx)).y;
reg_tmp7.z = (mul_s(reg_tmp2.yyyy, reg_tmp4.xxxx)).z;
reg_tmp7.z = (fma_s(reg_tmp4.yyyy, reg_tmp5.xxxx, reg_tmp7.zzzz)).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (mul_s(reg_tmp2.xxxx, reg_tmp5.xxxx)).x;
reg_tmp8.x = (fma_s(reg_tmp5.yyyy, reg_tmp4.xxxx, reg_tmp8.xxxx)).x;
reg_tmp8.y = (mul_s(-reg_tmp4.zzzz, reg_tmp5.xxxx)).y;
reg_tmp8.z = (mul_s(-reg_tmp2.yyyy, reg_tmp5.xxxx)).z;
reg_tmp8.z = (fma_s(reg_tmp4.yyyy, reg_tmp4.xxxx, reg_tmp8.zzzz)).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
return false;
}
bool sub_2() {
reg_tmp2 = uniforms.f[85];
reg_tmp2.xz = (-uniforms.f[5 + addr_regs.x].xzzz + reg_tmp2.xzzz).xz;
reg_tmp2.yw = (uniforms.f[0].xxxx).yw;
reg_tmp3.x = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp3.x = rsq_s(reg_tmp3.x);
reg_tmp2.xyz = (mul_s(reg_tmp2.xyzz, reg_tmp3.xxxx)).xyz;
reg_tmp4 = uniforms.f[0].xzxx;
reg_tmp5.xyz = (mul_s(reg_tmp2.yzxx, reg_tmp4.zxyy)).xyz;
reg_tmp5.xyz = (fma_s(-reg_tmp4.yzxx, reg_tmp2.zxyy, reg_tmp5)).xyz;
reg_tmp3.x = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp3.x = rsq_s(reg_tmp3.x);
reg_tmp5.xyz = (mul_s(reg_tmp5.xyzz, reg_tmp3.xxxx)).xyz;
reg_tmp6.x = (-reg_tmp5.xxxx).x;
reg_tmp6.y = (-reg_tmp5.yyyy).y;
reg_tmp6.z = (-reg_tmp5.zzzz).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (reg_tmp4.xxxx).x;
reg_tmp7.y = (reg_tmp4.yyyy).y;
reg_tmp7.z = (reg_tmp4.zzzz).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (-reg_tmp2.xxxx).x;
reg_tmp8.y = (-reg_tmp2.yyyy).y;
reg_tmp8.z = (-reg_tmp2.zzzz).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9.x = (uniforms.f[90].wwww).x;
reg_tmp9.y = (uniforms.f[91].wwww).y;
reg_tmp9.z = (uniforms.f[92].wwww).z;
reg_tmp9.w = (uniforms.f[0].zzzz).w;
return false;
}
bool sub_6() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_7() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
sub_2();
reg_tmp14.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp14.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp14.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp14.w = dot_s(uniforms.f[0].xxxz, reg_tmp10);
reg_tmp0 = uniforms.f[7 + addr_regs.x];
sub_3();
reg_tmp2.x = dot_s(reg_tmp14, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp14, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp14, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp14, reg_tmp9);
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[85].xyzz + -reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_4();
} else {
sub_5();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (uniforms.f[5 + addr_regs.x].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_6();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_7();
sub_8();
return true;
}
bool sub_4() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_5() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: 4616E61AEEE6669A, B53793A9026DE268
// program: B53793A9026DE268, D9AC2D854F4D2F3B, 94A9CEB53BFBA601
// shader: 8B30, B440A5B5F68EC249
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C3E4FCF292E299D3, B440A5B5F68EC249
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, B440A5B5F68EC249
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, D715F9B9FD91CE99
// shader: 8B31, 2F45AEBED29B4BE3

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp15 = uniforms.f[92].xxxz;
reg_tmp15.xyz = (vs_in_reg0).xyz;
reg_tmp10 = vs_in_reg3;
reg_tmp0.x = dot_s(uniforms.f[12], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[13], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[14], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[15], reg_tmp15);
vs_out_attr0 = reg_tmp0;
vs_out_attr1 = vs_in_reg1;
reg_tmp10 = vs_in_reg2;
reg_tmp10.xy = (uniforms.f[20].xyyy + reg_tmp10.xyyy).xy;
vs_out_attr2 = reg_tmp10;
reg_tmp10.xy = (uniforms.f[21].yxxx).xy;
vs_out_attr3 = reg_tmp10;
return true;
}
// reference: A0CFFB5CED4F1D36, 2F45AEBED29B4BE3
// shader: 8B30, 1EE6A82034608DE1
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.aaa) * (texcolor1.aaa) + (texcolor2.rgb) * (vec3(1) - (texcolor1.aaa)), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DCB59E45414424DF, 1EE6A82034608DE1
// program: 2F45AEBED29B4BE3, 64BE452CE0E53833, 1EE6A82034608DE1
// shader: 8B30, 10DD52F8D9E718B7
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1C305ED59EEC7236, 10DD52F8D9E718B7
// program: 2F45AEBED29B4BE3, 64BE452CE0E53833, 10DD52F8D9E718B7
// program: E1376E530AE24D2A, AF3D45D1FD56EDEB, E26DD8B9DA6EA50B
// shader: 8B30, 45F6291056F205AF
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5A2AB70556CD1705, 45F6291056F205AF
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 45F6291056F205AF
// shader: 8B30, 5391D11166690BD0
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (const_color[1].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C500A18964CAA298, 5391D11166690BD0
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 5391D11166690BD0
// reference: 969832D485B74DBA, 438F449963BC8B0F
// program: C370F5A1D7E96057, D9AC2D854F4D2F3B, 8F3D8BBC3BFBA601
// program: C370F5A1D7E96057, D9AC2D854F4D2F3B, D715F9B9E6058B90
// shader: 8B30, 5391D111A3729833
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) + (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (const_color[1].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2B2C716E0C76AB9E, 5391D111A3729833
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 5391D111A3729833
// shader: 8B30, 8B2DA80FBAD76617
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6706B2CA0BF16E83, 8B2DA80FBAD76617
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 8B2DA80FBAD76617
// shader: 8B30, 052F43E5E9AF1C9F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8E4F8A12F23F836C, 052F43E5E9AF1C9F
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, 052F43E5E9AF1C9F
// shader: 8B30, 7AD135E0FD22521F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (vec3(1) - texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7968AE274B34DB0E, 7AD135E0FD22521F
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 7AD135E0FD22521F
// shader: 8B30, D1EAF085C16876A8
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E2197A279EDE8, D1EAF085C16876A8
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, D1EAF085C16876A8
// reference: 5616CA3E52712BAD, 3EF703BBC60081BA
// shader: 8B30, B4BC20E1BD4DECE1
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (vec3(1) - texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5616CA3E33CBD57F, B4BC20E1BD4DECE1
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, B4BC20E1BD4DECE1
// shader: 8B30, 607F717A2CE2FC63
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D2924E9761775426, 607F717A2CE2FC63
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 607F717A2CE2FC63
// shader: 8B30, E0C90FD7ACFD2CDF
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rrr) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4225A15C3BF47733, E0C90FD7ACFD2CDF
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, E0C90FD7ACFD2CDF
// shader: 8B30, 363919041B99023A
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5A2AB705BFC60E39, 363919041B99023A
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 363919041B99023A
// shader: 8B30, 4B285B1C2453A6BA
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 57308F2769952323, 4B285B1C2453A6BA
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, 4B285B1C2453A6BA
// shader: 8B30, 3AB6BC4A72E547B0
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E2197D2FF6E0F, 3AB6BC4A72E547B0
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 3AB6BC4A72E547B0
// reference: 349E2197A50E083E, 3AB6BC4A72E547B0
// shader: 8B30, 00E4E5ED0CF5EF62
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3 * 2.0, alpha_output_3 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa) + (const_color[5].rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5097D0E9DE689C4A, 00E4E5ED0CF5EF62
// program: 3AE5BAF463299370, F3E3101E81A13CCB, 00E4E5ED0CF5EF62
// reference: EC2C2EDA3C8AD9D9, D715F9B9E6058B90
// reference: 6FC6753B3C8AD9D9, D715F9B9FD91CE99
// reference: 4A57EC4FCDF2C6C5, 165E9866130E3B99
// reference: AA9DF79758DC6A40, B440A5B5F68EC249
// program: C370F5A1D7E96057, D9AC2D854F4D2F3B, 20450603514F996E
// reference: B5CC95208B7AD74C, 1EE6A82034608DE1
// program: C370F5A1D7E96057, D9AC2D854F4D2F3B, B440A5B5F68EC249
// shader: 8B30, 0632F50210137226
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
vec3 color_output_4 = (const_color[4].rgb);
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 003107588B0B9975, 0632F50210137226
// program: A8402D5D722D637F, 64BE452CE0E53833, 0632F50210137226
// shader: 8B30, F7E4241F56C24D71
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ABB4371BC81634AF, F7E4241F56C24D71
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, F7E4241F56C24D71
// reference: C500A1896E3871B9, 50CA766563BC8B0F
// reference: C500A1890C76AB9E, 5391D11166690BD0
// shader: 8B30, 7EEDCF126F76A809
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((last_tex_env_out.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C500A1892E7354AB, 7EEDCF126F76A809
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 7EEDCF126F76A809
// shader: 8B31, F4A3B6BD277A4BA7

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
bool sub_11();
bool sub_2();
bool sub_9();
bool sub_10();
bool sub_0();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_11() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_2() {
reg_tmp6.xyz = (-reg_tmp0.xyzz).xyz;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.xyz = vec3(rcp_s(uniforms.f[81].y));
reg_tmp8.xyz = (fma_s(reg_tmp6.xyzz, reg_tmp7.xyzz, uniforms.f[0].yyyy)).xyz;
reg_tmp7.xyz = (floor(reg_tmp8.xyzz)).xyz;
reg_tmp6.xyz = (fma_s(reg_tmp7.xyzz, -uniforms.f[81].yyyy, reg_tmp6.xyzz)).xyz;
reg_tmp6 = min_s(uniforms.f[81].xxxx, reg_tmp6);
reg_tmp6 = max_s(-uniforms.f[81].xxxx, reg_tmp6);
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.xxxx)).y;
reg_tmp4.x = (reg_tmp1.xxxx).x;
reg_tmp5.x = (reg_tmp1.yyyy).x;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.yyyy)).y;
reg_tmp4.y = (reg_tmp1.xxxx).y;
reg_tmp5.y = (reg_tmp1.yyyy).y;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp6.zzzz, reg_tmp6.zzzz)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp6.zzzz)).y;
reg_tmp4.z = (reg_tmp1.xxxx).z;
reg_tmp5.z = (reg_tmp1.yyyy).z;
reg_tmp2.x = (mul_s(reg_tmp4.yyyy, reg_tmp5.zzzz)).x;
reg_tmp2.y = (mul_s(reg_tmp5.yyyy, reg_tmp5.zzzz)).y;
reg_tmp6.x = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).x;
reg_tmp6.y = (reg_tmp5.zzzz).y;
reg_tmp6.z = (mul_s(-reg_tmp5.yyyy, reg_tmp4.zzzz)).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (mul_s(-reg_tmp2.xxxx, reg_tmp4.xxxx)).x;
reg_tmp7.x = (fma_s(reg_tmp5.yyyy, reg_tmp5.xxxx, reg_tmp7.xxxx)).x;
reg_tmp7.y = (mul_s(reg_tmp4.zzzz, reg_tmp4.xxxx)).y;
reg_tmp7.z = (mul_s(reg_tmp2.yyyy, reg_tmp4.xxxx)).z;
reg_tmp7.z = (fma_s(reg_tmp4.yyyy, reg_tmp5.xxxx, reg_tmp7.zzzz)).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (mul_s(reg_tmp2.xxxx, reg_tmp5.xxxx)).x;
reg_tmp8.x = (fma_s(reg_tmp5.yyyy, reg_tmp4.xxxx, reg_tmp8.xxxx)).x;
reg_tmp8.y = (mul_s(-reg_tmp4.zzzz, reg_tmp5.xxxx)).y;
reg_tmp8.z = (mul_s(-reg_tmp2.yyyy, reg_tmp5.xxxx)).z;
reg_tmp8.z = (fma_s(reg_tmp4.yyyy, reg_tmp4.xxxx, reg_tmp8.zzzz)).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
return false;
}
bool sub_9() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_10() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
reg_tmp14 = reg_tmp10;
reg_tmp0 = uniforms.f[7 + addr_regs.x];
sub_2();
reg_tmp10.x = dot_s(reg_tmp14, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp14, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp14, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp14, reg_tmp9);
reg_tmp2.x = (uniforms.f[8 + addr_regs.x].yyyy).x;
reg_tmp2.y = (uniforms.f[9 + addr_regs.x].yyyy).y;
reg_tmp2.z = (uniforms.f[10 + addr_regs.x].yyyy).z;
reg_tmp3.xyz = (uniforms.f[6 + addr_regs.x].xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp4.xyz = (mul_s(reg_tmp3.yzxx, reg_tmp2.zxyy)).xyz;
reg_tmp4.xyz = (fma_s(-reg_tmp2.yzxx, reg_tmp3.zxyy, reg_tmp4)).xyz;
reg_tmp5.x = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp5.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp5.y;
if (bool_regs.x) {
sub_5();
} else {
sub_6();
}
reg_tmp4.xyz = (mul_s(reg_tmp4.xyzz, reg_tmp5.xxxx)).xyz;
reg_tmp5.xyz = (mul_s(reg_tmp3.yzxx, reg_tmp4.zxyy)).xyz;
reg_tmp5.xyz = (fma_s(-reg_tmp4.yzxx, reg_tmp3.zxyy, reg_tmp5)).xyz;
reg_tmp6.x = (reg_tmp4.xxxx).x;
reg_tmp6.y = (reg_tmp3.xxxx).y;
reg_tmp6.z = (reg_tmp5.xxxx).z;
reg_tmp6.w = (uniforms.f[0].xxxx).w;
reg_tmp7.x = (reg_tmp4.yyyy).x;
reg_tmp7.y = (reg_tmp3.yyyy).y;
reg_tmp7.z = (reg_tmp5.yyyy).z;
reg_tmp7.w = (uniforms.f[0].xxxx).w;
reg_tmp8.x = (reg_tmp4.zzzz).x;
reg_tmp8.y = (reg_tmp3.zzzz).y;
reg_tmp8.z = (reg_tmp5.zzzz).z;
reg_tmp8.w = (uniforms.f[0].xxxx).w;
reg_tmp9 = uniforms.f[0].xxxz;
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp10, reg_tmp9);
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[85].xyzz + -reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_7();
} else {
sub_8();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (uniforms.f[5 + addr_regs.x].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_9();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_10();
sub_11();
return true;
}
bool sub_3() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_4() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
bool sub_5() {
reg_tmp5.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_6() {
reg_tmp5.x = rsq_s(reg_tmp5.x);
return false;
}
bool sub_7() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_8() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: 0B863F1279C8BB0D, F4A3B6BD277A4BA7
// program: F4A3B6BD277A4BA7, D9AC2D854F4D2F3B, 4B285B1C2453A6BA
// shader: 8B30, 84A092105756925D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6706B2CAC8376808, 84A092105756925D
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 84A092105756925D
// shader: 8B31, 67B3BBF0B3D00945

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
bool sub_6();
bool sub_4();
bool sub_5();
bool sub_0();
bool sub_2();
bool sub_3();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
addr_regs.x = (ivec2(vs_in_reg0.zz)).x;
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xy = (vs_in_reg0.xyyy).xy;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_6() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = reg_tmp13;
reg_tmp11.y = (uniforms.f[0].zzzz + -reg_tmp11.yyyy).y;
reg_tmp12.y = (uniforms.f[0].zzzz + -reg_tmp12.yyyy).y;
reg_tmp14.y = (uniforms.f[81].wwww).y;
reg_tmp14.x = (mul_s(uniforms.f[0].wwww, reg_tmp14.yyyy)).x;
reg_tmp2.x = rcp_s(reg_tmp14.x);
reg_tmp14.z = (reg_tmp2.xxxx).z;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp11.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp11.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
reg_tmp3.xy = (uniforms.f[81].wwww + reg_tmp12.xyyy).xy;
reg_tmp5.xy = (mul_s(reg_tmp3.xyyy, reg_tmp14.zzzz)).xy;
reg_tmp4.xy = (floor(reg_tmp5.xyyy)).xy;
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp14.xxxx)).xy;
reg_tmp4.xy = (reg_tmp3.xyyy + -reg_tmp4.xyyy).xy;
reg_tmp12.xy = (-uniforms.f[81].wwww + reg_tmp4.xyyy).xy;
vs_out_attr2 = reg_tmp11;
vs_out_attr3 = reg_tmp12;
return false;
}
bool sub_4() {
reg_tmp2 = uniforms.f[90];
reg_tmp3 = uniforms.f[91];
reg_tmp4 = uniforms.f[92];
reg_tmp5 = uniforms.f[0].xxxz;
reg_tmp6 = mul_s(uniforms.f[86].xxxx, reg_tmp2);
reg_tmp6 = fma_s(reg_tmp3, uniforms.f[86].yyyy, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp4, uniforms.f[86].zzzz, reg_tmp6);
reg_tmp6 = fma_s(reg_tmp5, uniforms.f[86].wwww, reg_tmp6);
reg_tmp7 = mul_s(uniforms.f[87].xxxx, reg_tmp2);
reg_tmp7 = fma_s(reg_tmp3, uniforms.f[87].yyyy, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp4, uniforms.f[87].zzzz, reg_tmp7);
reg_tmp7 = fma_s(reg_tmp5, uniforms.f[87].wwww, reg_tmp7);
reg_tmp8 = mul_s(uniforms.f[88].xxxx, reg_tmp2);
reg_tmp8 = fma_s(reg_tmp3, uniforms.f[88].yyyy, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp4, uniforms.f[88].zzzz, reg_tmp8);
reg_tmp8 = fma_s(reg_tmp5, uniforms.f[88].wwww, reg_tmp8);
reg_tmp9 = mul_s(uniforms.f[89].xxxx, reg_tmp2);
reg_tmp9 = fma_s(reg_tmp3, uniforms.f[89].yyyy, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp4, uniforms.f[89].zzzz, reg_tmp9);
reg_tmp9 = fma_s(reg_tmp5, uniforms.f[89].wwww, reg_tmp9);
return false;
}
bool sub_5() {
reg_tmp2.x = rcp_s(uniforms.f[81].y);
reg_tmp2.xy = (mul_s(uniforms.f[2 + addr_regs.x].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2 + addr_regs.x].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, -reg_tmp2.xyyy)).xy;
reg_tmp0.xy = (max_s(-uniforms.f[81].xxxx, reg_tmp0.xyyy)).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.xxxx, reg_tmp0.xxxx)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.xxxx)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].xxxx).z;
reg_tmp4.w = (-uniforms.f[83].yyyy).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[3 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[3 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[3 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp11.xy = (reg_tmp5.xyyy).xy;
reg_tmp2 = uniforms.f[95];
reg_tmp3 = uniforms.f[94];
reg_tmp1.z = (mul_s(reg_tmp0.yyyy, reg_tmp0.yyyy)).z;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp3.zwww)).xy;
reg_tmp2 = uniforms.f[93];
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.xyyy)).xy;
reg_tmp1.xy = (fma_s(reg_tmp1.zzzz, reg_tmp1.xyyy, reg_tmp2.zwww)).xy;
reg_tmp1.y = (mul_s(reg_tmp1.yyyy, reg_tmp0.yyyy)).y;
reg_tmp4.xy = (mul_s(vs_in_reg0.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-vs_in_reg0.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(vs_in_reg0.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
reg_tmp5.x = (uniforms.f[0].yyyy + reg_tmp4.xxxx).x;
reg_tmp5.y = (-uniforms.f[0].yyyy + reg_tmp4.yyyy).y;
reg_tmp4.z = (uniforms.f[83].zzzz).z;
reg_tmp4.w = (-uniforms.f[83].wwww).w;
reg_tmp5.xy = (fma_s(reg_tmp5.xyyy, reg_tmp4.zwww, uniforms.f[4 + addr_regs.x].xyyy)).xy;
reg_tmp4.z = (-uniforms.f[4 + addr_regs.x].zzzz).z;
reg_tmp4.w = (-uniforms.f[4 + addr_regs.x].wwww).w;
reg_tmp5.z = (uniforms.f[0].zzzz + reg_tmp4.zzzz).z;
reg_tmp5.w = (uniforms.f[0].zzzz + reg_tmp4.wwww).w;
reg_tmp5.x = (fma_s(-reg_tmp5.zzzz, reg_tmp4.xxxx, reg_tmp5.xxxx)).x;
reg_tmp5.y = (fma_s(reg_tmp5.wwww, reg_tmp4.yyyy, reg_tmp5.yyyy)).y;
reg_tmp12.xy = (reg_tmp5.xyyy).xy;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xy = (mul_s(uniforms.f[2 + addr_regs.x].xyyy, reg_tmp10.xyyy)).xy;
reg_tmp6 = uniforms.f[8 + addr_regs.x];
reg_tmp7 = uniforms.f[9 + addr_regs.x];
reg_tmp8 = uniforms.f[10 + addr_regs.x];
reg_tmp9 = uniforms.f[0].xxxz;
reg_tmp6.w = (uniforms.f[5 + addr_regs.x].xxxx).w;
reg_tmp7.w = (uniforms.f[5 + addr_regs.x].yyyy).w;
reg_tmp8.w = (uniforms.f[5 + addr_regs.x].zzzz).w;
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp10, reg_tmp9);
reg_tmp10 = reg_tmp2;
reg_tmp3.xyz = (uniforms.f[5 + addr_regs.x].xyzz).xyz;
reg_tmp3.xyz = (uniforms.f[85].xyzz + -reg_tmp3.xyzz).xyz;
reg_tmp4.x = dot_3(reg_tmp3.xyz, reg_tmp3.xyz);
bool_regs.x = uniforms.f[0].xxxx.x >= reg_tmp4.x;
bool_regs.y = uniforms.f[0].xxxx.y == reg_tmp4.y;
if (bool_regs.x) {
sub_2();
} else {
sub_3();
}
reg_tmp3.xyz = (mul_s(reg_tmp3.xyzz, reg_tmp4.xxxx)).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_4();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1 + addr_regs.x];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_5();
sub_6();
return true;
}
bool sub_2() {
reg_tmp4.x = (uniforms.f[0].zzzz).x;
return false;
}
bool sub_3() {
reg_tmp4.x = rsq_s(reg_tmp4.x);
return false;
}
// reference: 6FAD57A2086AE351, 67B3BBF0B3D00945
// program: 67B3BBF0B3D00945, D9AC2D854F4D2F3B, B8762D381189F738
// program: 67B3BBF0B3D00945, D9AC2D854F4D2F3B, 8F3D8BBC3BFBA601
// program: 5E172D0FA1530810, AF3D45D1FD56EDEB, 7F455826CF3F18FD
// shader: 8B31, B665EB15EB82752B

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
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp6.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp6.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp6.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12.x = dot_3(uniforms.f[25].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27].xyz, reg_tmp14.xyz);
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_5() {
reg_tmp1 = vec4(dot_3(uniforms.f[4].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[4].wwww;
reg_tmp9 = vs_in_reg3;
reg_tmp0.y = (uniforms.f[7].wwww).y;
reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9.w = (uniforms.f[21].wwww).w;
reg_tmp3 = uniforms.f[22];
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp2 = uniforms.f[5] + -reg_tmp3;
if (bool_regs.y) {
sub_6();
}
reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
if (uniforms.b[6]) {
sub_7();
}
reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
vs_out_attr3 = reg_tmp9;
return false;
}
bool sub_6() {
reg_tmp9 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp9.xyzz)).xyz;
return false;
}
bool sub_7() {
reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
return false;
}
bool sub_8() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
return false;
}
bool sub_0() {
sub_1();
sub_2();
sub_5();
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
vs_out_attr6 = reg_tmp6.xyyy;
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
sub_8();
vs_out_attr4 = reg_tmp3.xyyy;
vs_out_attr5 = reg_tmp6.xyyy;
return true;
}
// reference: 1703B5A944D5479B, B665EB15EB82752B
// shader: 8B30, B963F22E0518940F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) + (((lut_scale_d1 * LookupLightingLUTSigned(1, dot(light_vector, normal))) * light_src[0].specular_1) * geo_factor)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 94B168155DCF7F2D, B963F22E0518940F
// program: B665EB15EB82752B, F3E3101E81A13CCB, B963F22E0518940F
// shader: 8B30, DE1B874A1C8A5017
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa) + (const_color[3].rgb) * (vec3(1) - (const_color[3].aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6E5F6BDE0278B3F5, DE1B874A1C8A5017
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, DE1B874A1C8A5017
// shader: 8B30, D5CEA35487DBB831
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 456E4B32448D9A2F, D5CEA35487DBB831
// program: B665EB15EB82752B, F3E3101E81A13CCB, D5CEA35487DBB831
// shader: 8B30, B4521DB926F9E3D4
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1781C1E59D89BBC6, B4521DB926F9E3D4
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, B4521DB926F9E3D4
// shader: 8B30, 00BD27EE68CE2F45
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1781C1E57ED4B7D9, 00BD27EE68CE2F45
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 00BD27EE68CE2F45
// shader: 8B30, 3ED5D12E909314A2
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3 * 2.0, alpha_output_3 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa) + (const_color[5].rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0E7D3EAD6C992FD6, 3ED5D12E909314A2
// program: 3AE5BAF463299370, F3E3101E81A13CCB, 3ED5D12E909314A2
// shader: 8B30, 219879F272E547B0
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6706B2CAD2FF6E0F, 219879F272E547B0
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 219879F272E547B0
// shader: 8B30, 5616E74CC542ECBB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0031075855D5726B, 5616E74CC542ECBB
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 5616E74CC542ECBB
// shader: 8B30, 6A8468B70E64568E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (1.0 - texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A3727844BFC60E39, 6A8468B70E64568E
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 6A8468B70E64568E
// shader: 8B30, B6E33F04013A26EC
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9B36DD81E7664D45, B6E33F04013A26EC
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, B6E33F04013A26EC
// program: F4A3B6BD277A4BA7, D9AC2D854F4D2F3B, 94A9CEB53BFBA601
// shader: 8B30, 47E1F930C6AFA543
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5E6D5F2D8330FEDC, 47E1F930C6AFA543
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, 47E1F930C6AFA543
// shader: 8B30, 57D4B1FD6306AC70
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8E02C5A2B34590DD, 57D4B1FD6306AC70
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 57D4B1FD6306AC70
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
// shader: 8B31, 8F1ACE4866373AD1

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
reg_tmp15 = uniforms.f[92].xxxz;
reg_tmp15.xyz = (vs_in_reg0.xyzz).xyz;
vs_out_attr0.x = dot_s(uniforms.f[12], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[13], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[14], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[15], reg_tmp15);
vs_out_attr1 = vs_in_reg1.xyyy;
return true;
}
// reference: 994C9B7FF275056C, 8F1ACE4866373AD1
// shader: 8B30, DABFACD322D80123
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (texcolor0.ggg) + (const_color[0].rrr), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa) + (texcolor0.rgb) * (vec3(1) - (const_color[5].aaa)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3CB1E16241C97044, DABFACD322D80123
// program: 8F1ACE4866373AD1, 6F6B8D04DE5400F0, DABFACD322D80123
// shader: 8B30, FA86933288A68C55
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa) + (const_color[1].rgb) * (vec3(1) - (const_color[1].aaa)), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a) + (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4636432A87977BD9, FA86933288A68C55
// program: F690B9DE63154125, 422B94F1339C42ED, FA86933288A68C55
// shader: 8B30, 0CA02044D3E487AF
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C3E4FCF2C4C00181, 0CA02044D3E487AF
// program: 2D8E060A79F7CD57, D9AC2D854F4D2F3B, 0CA02044D3E487AF
// reference: 5A2AB705774FB805, 363919041B99023A
// reference: 94B1681561A72BBF, 4271F240369D5DD5
// shader: 8B30, D867F3B7B09136D8
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.g), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7AAB7045BC4BF89D, D867F3B7B09136D8
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, D867F3B7B09136D8
// shader: 8B30, 17EBD0E543DAC901
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.g), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7AAB7045D2FF6E0F, 17EBD0E543DAC901
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 17EBD0E543DAC901
// shader: 8B30, CEE5FF5197732398
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A70021E2BB3C1D4B, CEE5FF5197732398
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, CEE5FF5197732398
// shader: 8B30, BA3F1707302DF02D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.g), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AB745362251CD730, BA3F1707302DF02D
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, BA3F1707302DF02D
// program: A1BD3EE2FA292D0C, D9AC2D854F4D2F3B, 052F43E5E9AF1C9F
// shader: 8B30, 96D53447F115ABEC
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F64B4FD4D6DE1551, 96D53447F115ABEC
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 96D53447F115ABEC
// shader: 8B30, A55D4FB568791A0B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CA3F66A9387E8DF0, A55D4FB568791A0B
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, A55D4FB568791A0B
// shader: 8B30, 12B5EE25D3534FA0
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normalize(view), normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E353BCF6E039EC8E, 12B5EE25D3534FA0
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 12B5EE25D3534FA0
// reference: E353BCF65C4E8F32, 12B5EE25D3534FA0
// program: BDF7416D65A6B66F, F3E3101E81A13CCB, 12B5EE25D3534FA0
// shader: 8B31, 3F944AA773C1E508

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
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

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
bool sub_4();
bool sub_0();
bool sub_5();
bool sub_6();
bool sub_7();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
addr_regs.xy = ivec2(reg_tmp2.xy);
reg_tmp1 = mul_s(uniforms.f[8].wwww, vs_in_reg8);
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp3.x = dot_s(uniforms.f[23 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[24 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[23 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[24 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp2.x = dot_s(uniforms.f[23 + addr_regs.y], reg_tmp15);
reg_tmp2.y = dot_s(uniforms.f[24 + addr_regs.y], reg_tmp15);
reg_tmp2.z = dot_s(uniforms.f[25 + addr_regs.y], reg_tmp15);
reg_tmp5.x = dot_3(uniforms.f[23 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.y = dot_3(uniforms.f[24 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.z = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp6 = mul_s(reg_tmp1.xxxx, reg_tmp3);
reg_tmp12 = mul_s(reg_tmp1.xxxx, reg_tmp4);
reg_tmp6 = fma_s(reg_tmp1.yyyy, reg_tmp2, reg_tmp6);
reg_tmp12 = fma_s(reg_tmp1.yyyy, reg_tmp5, reg_tmp12);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_0() {
sub_1();
sub_2();
sub_5();
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
vs_out_attr4 = reg_tmp3.xyyy;
vs_out_attr5 = reg_tmp6.xyyy;
vs_out_attr6 = reg_tmp5.xyyy;
return true;
}
bool sub_5() {
reg_tmp1 = vec4(dot_3(uniforms.f[4].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[4].wwww;
reg_tmp9 = vs_in_reg3;
reg_tmp0.y = (uniforms.f[7].wwww).y;
reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9.w = (uniforms.f[21].wwww).w;
reg_tmp3 = uniforms.f[22];
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp2 = uniforms.f[5] + -reg_tmp3;
if (bool_regs.y) {
sub_6();
}
reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
if (uniforms.b[6]) {
sub_7();
}
reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
vs_out_attr3 = reg_tmp9;
return false;
}
bool sub_6() {
reg_tmp9 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp9.xyzz)).xyz;
return false;
}
bool sub_7() {
reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
return false;
}
// reference: 1703B5A936FF2AEF, 3F944AA773C1E508
// shader: 8B30, 034FCC5754DAED51
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) * geo_factor) + ((lut_scale_d1 * LookupLightingLUTSigned(1, dot(normal, normalize(view)))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa) + (const_color[3].rgb) * (vec3(1) - (const_color[3].aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CC1C4DCBA887D336, 034FCC5754DAED51
// program: 3F944AA773C1E508, F3E3101E81A13CCB, 034FCC5754DAED51
// shader: 8B30, 99391860D08E88F1
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) * geo_factor) + ((lut_scale_d1 * LookupLightingLUTSigned(1, dot(normal, normalize(view)))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor1.b);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.b);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.b);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.b);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa) + (const_color[4].rgb) * (vec3(1) - (const_color[4].aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.b);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 82D00270E1898979, 99391860D08E88F1
// program: 3AE5BAF463299370, F3E3101E81A13CCB, 99391860D08E88F1
// shader: 8B30, EAD4A81852C25F2D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) * geo_factor) + ((lut_scale_d1 * LookupLightingLUTSigned(1, dot(normal, normalize(view)))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa) + (const_color[3].rgb) * (vec3(1) - (const_color[3].aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FB8FA8EAA887D336, EAD4A81852C25F2D
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, EAD4A81852C25F2D
// shader: 8B30, 20C295A9A4E134FC
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.b) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa) + (const_color[2].rgb) * (vec3(1) - (const_color[2].aaa)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A86058BDEA835D49, 20C295A9A4E134FC
// program: 3F944AA773C1E508, F3E3101E81A13CCB, 20C295A9A4E134FC
// shader: 8B30, 535FE4CE198B91FD
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa) + (const_color[2].rgb) * (vec3(1) - (const_color[2].aaa)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 275D7A634CBE7DE7, 535FE4CE198B91FD
// program: A0B8370D113E7E49, F3E3101E81A13CCB, 535FE4CE198B91FD
// shader: 8B30, 1ADF75003EBD80ED
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) + (((lut_scale_d1 * LookupLightingLUTSigned(1, dot(light_vector, normal))) * light_src[0].specular_1) * geo_factor)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa) + (const_color[3].rgb) * (vec3(1) - (const_color[3].aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 06CFC34491C65769, 1ADF75003EBD80ED
// program: 85E327F85CBA222D, F3E3101E81A13CCB, 1ADF75003EBD80ED
// program: 3F944AA773C1E508, F3E3101E81A13CCB, EAD4A81852C25F2D
// shader: 8B30, 1FF4FCD3270B697B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa) + (const_color[2].rgb) * (vec3(1) - (const_color[2].aaa)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 08231E7ABFE0252A, 1FF4FCD3270B697B
// program: 3F944AA773C1E508, F3E3101E81A13CCB, 1FF4FCD3270B697B
// shader: 8B30, DF06C9F4955BCD97
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor1.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor1.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor1.g), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7E8983DABE0F85A8, DF06C9F4955BCD97
// program: A647F14B0179FC4B, F3E3101E81A13CCB, DF06C9F4955BCD97
// shader: 8B30, 37AF62101D826638
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 97F5BD36451337DC, 37AF62101D826638
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 37AF62101D826638
// shader: 8B30, 9F9FD1E5E2854CAD
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.g);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor1.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 101E017D49AF7BE7, 9F9FD1E5E2854CAD
// program: A647F14B0179FC4B, F3E3101E81A13CCB, 9F9FD1E5E2854CAD
// shader: 8B30, 23549632D14552DB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor2.rgb) + (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor2.ggg) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BC7CA5A35975FA3F, 23549632D14552DB
// program: A647F14B0179FC4B, F3E3101E81A13CCB, 23549632D14552DB
// shader: 8B31, 6A58AFD5AF2100E8

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
layout(location=6) in vec4 vs_in_reg6;

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
bool sub_4();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp6.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp6.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp6.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12.x = dot_3(uniforms.f[25].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27].xyz, reg_tmp14.xyz);
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_0() {
sub_1();
sub_2();
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp8.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
reg_tmp9.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp8.zw = (uniforms.f[93].xxyy).zw;
reg_tmp9.zw = (uniforms.f[93].xxyy).zw;
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp8);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp8);
reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp9);
reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp9);
reg_tmp9 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
vs_out_attr4 = reg_tmp3.xyyy;
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp9.xyzz)).xyz;
vs_out_attr5 = reg_tmp4;
vs_out_attr3 = reg_tmp9;
vs_out_attr6 = reg_tmp5;
return true;
}
// reference: 1703B5A9229E08A4, 6A58AFD5AF2100E8
// shader: 8B30, 3C161F5DAB3931FB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.aaa) + (const_color[0].rgb) * (vec3(1) - (texcolor1.aaa)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.bbb) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor1.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) - (texcolor2.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7A3DACEE3FB511E6, 3C161F5DAB3931FB
// program: 6A58AFD5AF2100E8, F3E3101E81A13CCB, 3C161F5DAB3931FB
// shader: 8B30, A3059407569A07BF
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.aaa) + (const_color[0].rgb) * (vec3(1) - (texcolor1.aaa)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.g), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor1.bbb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1C1FEA9CA52EDBCF, A3059407569A07BF
// program: A647F14B0179FC4B, F3E3101E81A13CCB, A3059407569A07BF
// shader: 8B30, E6C56F51149C32B5
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normalize(view), normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) + (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C527DC1E2786A509, E6C56F51149C32B5
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, E6C56F51149C32B5
// shader: 8B30, AB4122B6AA36A0F8
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B40667E21DACE05E, AB4122B6AA36A0F8
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, AB4122B6AA36A0F8
// shader: 8B30, 9533CA2D514F996E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0DA5D1F3A41D1B3E, 9533CA2D514F996E
// program: C370F5A1D7E96057, D9AC2D854F4D2F3B, 9533CA2D514F996E
// shader: 8B30, 1E4A5DCAA304CE63
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (const_color[1].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ABB4371B7B87CDAF, 1E4A5DCAA304CE63
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, 1E4A5DCAA304CE63
// reference: F82CA4461E1489F6, 1E4A5DCAA304CE63
// shader: 8B31, C365BBC9CAE5768A

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
bool sub_4();
bool sub_5();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp15.xyz = (uniforms.f[6].xyzz + reg_tmp15.xyzz).xyz;
reg_tmp6.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp6.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp6.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp6.w = (uniforms.f[93].yyyy).w;
reg_tmp12.x = dot_3(uniforms.f[25].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27].xyz, reg_tmp14.xyz);
reg_tmp6.xyz = (uniforms.f[95].xyzz + reg_tmp6.xyzz).xyz;
reg_tmp6.z = (mul_s(uniforms.f[95].wwww, reg_tmp6.zzzz)).z;
return false;
}
bool sub_2() {
uint jmp_to = 78u;
while (true) {
switch (jmp_to) {
case 78u:
reg_tmp6.w = (uniforms.f[93].yyyy).w;
bool_regs = equal(uniforms.f[9].xy, reg_tmp6.ww);
if (bool_regs.x) {
sub_3();
} else {
sub_4();
}
vs_out_attr0 = reg_tmp13;
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp0 = uniforms.f[93].yxxx;
reg_tmp6.x = (uniforms.f[94].yyyy + reg_tmp6.xxxx).x;
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp1 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp1 = mul_s(uniforms.f[94].zzzz, reg_tmp1);
reg_tmp2 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp1.xx);
reg_tmp1 = vec4(rsq_s(reg_tmp1.x));
if (bool_regs.x) {
jmp_to = 117u; break;
}
reg_tmp0.z = rcp_s(reg_tmp1.x);
reg_tmp0.xy = (mul_s(reg_tmp2, reg_tmp1)).xy;
case 117u:
vs_out_attr2 = -reg_tmp15;
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_3() {
reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_4() {
reg_tmp14.x = dot_3(uniforms.f[83].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[84].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[85].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[83], reg_tmp6);
reg_tmp15.y = dot_s(uniforms.f[84], reg_tmp6);
reg_tmp15.z = dot_s(uniforms.f[85], reg_tmp6);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
return false;
}
bool sub_5() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
return false;
}
bool sub_0() {
sub_1();
sub_2();
vs_out_attr3 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
sub_5();
vs_out_attr4 = reg_tmp3.xyyy;
vs_out_attr5 = reg_tmp6.xyyy;
vs_out_attr6 = reg_tmp5.xyyy;
return true;
}
// reference: 1703B5A98C9122B8, C365BBC9CAE5768A
// shader: 8B30, D2574C27B9F33324
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor1.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor1.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa) + (const_color[3].rgb) * (vec3(1) - (const_color[3].aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ADC7DFC4E54C4334, D2574C27B9F33324
// program: C365BBC9CAE5768A, F3E3101E81A13CCB, D2574C27B9F33324
// shader: 8B30, F8ECBB963A171CA2
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E0B65330D2FF6E0F, F8ECBB963A171CA2
// program: 22118D45A7CD8526, B24DF5DBCE8D1417, F8ECBB963A171CA2
// shader: 8B30, 03FD0C1F4A152B1D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor1.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4878D6D3BAE89831, 03FD0C1F4A152B1D
// program: A647F14B0179FC4B, F3E3101E81A13CCB, 03FD0C1F4A152B1D
// shader: 8B30, B440A5B543F80E67
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
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
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 1.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - combiner_buffer.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 400EA71392E299D3, B440A5B543F80E67
// program: C370F5A1D7E96057, D9AC2D854F4D2F3B, B440A5B543F80E67

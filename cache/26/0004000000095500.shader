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
// shader: 8B31, 5DFCA9186B211A41

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
bool sub_2();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp1.x = dot_s(uniforms.f[16], vs_in_reg0);
reg_tmp1.y = dot_s(uniforms.f[17], vs_in_reg0);
reg_tmp1.zw = (vs_in_reg0).zw;
reg_tmp0.x = (uniforms.f[11].wwww).x;
reg_tmp0.x = (mul_s(uniforms.f[25], reg_tmp0)).x;
reg_tmp1.x = (reg_tmp1 + reg_tmp0).x;
reg_tmp0.x = dot_s(uniforms.f[20], reg_tmp1);
reg_tmp0.y = dot_s(uniforms.f[21], reg_tmp1);
reg_tmp0.z = dot_s(uniforms.f[22], reg_tmp1);
reg_tmp0.w = dot_s(uniforms.f[23], reg_tmp1);
return false;
}
bool sub_0() {
sub_1();
reg_tmp1.w = (uniforms.f[11].wwww).w;
vs_out_attr0 = mul_s(uniforms.f[24].zzzz, reg_tmp0);
bool_regs.x = uniforms.f[95].xxxx.x < reg_tmp1.wwww.x;
bool_regs.y = uniforms.f[95].xxxx.y == reg_tmp1.wwww.y;
vs_out_attr1 = mul_s(uniforms.f[94].yyyy, vs_in_reg1);
reg_tmp1 = uniforms.f[19] + vs_in_reg2;
if (bool_regs.x) {
sub_2();
}
vs_out_attr2 = reg_tmp1;
return true;
}
bool sub_2() {
reg_tmp1.y = (uniforms.f[25] + reg_tmp1).y;
return false;
}
// reference: 115F4BB9099C12E6, 5DFCA9186B211A41
// shader: 8B30, 495157C00EA0C2B7
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
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E2197B224B5E3, 495157C00EA0C2B7
// program: 5DFCA9186B211A41, 8F66AB8FEF098E16, 495157C00EA0C2B7
// shader: 8B30, 1F1F5184595C7746
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
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FDAC8388E2FEBEAC, 1F1F5184595C7746
// program: 5DFCA9186B211A41, 8F66AB8FEF098E16, 1F1F5184595C7746
// reference: 349E2197D0C0146C, 495157C00EA0C2B7
// shader: 8B30, 53C984AEDA2AE735
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
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 591674EBAE275483, 53C984AEDA2AE735
// program: 5DFCA9186B211A41, 8F66AB8FEF098E16, 53C984AEDA2AE735
// shader: 8B30, E834550CD523D1C0
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
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 030DC4B6D0C0146C, E834550CD523D1C0
// program: 5DFCA9186B211A41, 8F66AB8FEF098E16, E834550CD523D1C0
// shader: 8DD9, 09CF57C48520A714

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

    texcoord0_w = vtx.attributes[2].z;
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
// reference: 83ABF08ECCF76B42, 09CF57C48520A714
// shader: 8B31, 394E558A7A83C1CC

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
bool sub_14();
bool sub_0();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_15();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp0.x = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp14);
reg_tmp0.y = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp14);
reg_tmp0.z = dot_s(uniforms.f[22 + addr_regs.x], reg_tmp14);
reg_tmp15.xyz = (fma_s(reg_tmp0.xyzz, reg_tmp13.wwww, reg_tmp15.xyzz)).xyz;
return false;
}
bool sub_14() {
reg_tmp0.x = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp0.y = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp0.z = dot_3(uniforms.f[22 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.xyz = (fma_s(reg_tmp0.xyzz, reg_tmp13.wwww, reg_tmp15.xyzz)).xyz;
return false;
}
bool sub_0() {
uint jmp_to = 35u;
while (true) {
switch (jmp_to) {
case 35u:
bool_regs = equal(uniforms.f[95].xx, vs_in_reg3.xy);
reg_tmp14 = mul_s(uniforms.f[94].zzzw, vs_in_reg0);
reg_tmp15 = reg_tmp14;
if (bool_regs.x) {
jmp_to = 56u; break;
}
addr_regs.x = (ivec2(vs_in_reg4.xx)).x;
reg_tmp13.w = (mul_s(uniforms.f[94].xxxx, vs_in_reg3.xxxx)).w;
reg_tmp15 = uniforms.f[95].xxxw;
sub_1();
if (bool_regs.y) {
jmp_to = 56u; break;
}
addr_regs.x = (ivec2(vs_in_reg4.yy)).x;
reg_tmp13.w = (mul_s(uniforms.f[94].xxxx, vs_in_reg3.yyyy)).w;
sub_1();
bool_regs = equal(uniforms.f[95].xx, vs_in_reg3.zw);
if (bool_regs.x) {
jmp_to = 56u; break;
}
addr_regs.x = (ivec2(vs_in_reg4.zz)).x;
reg_tmp13.w = (mul_s(uniforms.f[94].xxxx, vs_in_reg3.zzzz)).w;
sub_1();
if (bool_regs.y) {
jmp_to = 56u; break;
}
addr_regs.x = (ivec2(vs_in_reg4.ww)).x;
reg_tmp13.w = (mul_s(uniforms.f[94].xxxx, vs_in_reg3.wwww)).w;
sub_1();
case 56u:
if (uniforms.b[1]) {
sub_2();
} else {
sub_3();
}
reg_tmp0.w = (uniforms.f[95].wwww).w;
bool_regs.x = uniforms.f[64].xyyy.x < reg_tmp0.yyyy.x;
bool_regs.y = uniforms.f[64].xyyy.y > reg_tmp0.yyyy.y;
vs_out_attr1.xyz = (mul_s(uniforms.f[94].yyyy, vs_in_reg1.xyzz)).xyz;
if (any(bool_regs)) {
sub_4();
} else {
sub_7();
}
if (uniforms.b[3]) {
sub_8();
} else {
sub_9();
}
reg_tmp2.xy = (mul_s(uniforms.f[93].xyyy, -reg_tmp1.wwww)).xy;
bool_regs.x = reg_tmp1.xxxx.x < reg_tmp2.xyyy.x;
bool_regs.y = reg_tmp1.xxxx.y > reg_tmp2.xyyy.y;
if (all(bool_regs)) {
sub_10();
}
vs_out_attr0 = reg_tmp1;
if (uniforms.b[2]) {
sub_11();
} else {
sub_12();
}
vs_out_attr2 = reg_tmp12;
vs_out_attr3 = reg_tmp12;
return true;
default: return false;
}
}
return false;
}
bool sub_2() {
reg_tmp1 = uniforms.f[16];
reg_tmp2 = uniforms.f[17];
reg_tmp3 = uniforms.f[18];
reg_tmp1.w = (-uniforms.f[11].xxxx + reg_tmp1.wwww).w;
reg_tmp2.w = (-uniforms.f[11].yyyy + reg_tmp2.wwww).w;
reg_tmp3.w = (-uniforms.f[11].zzzz + reg_tmp3.wwww).w;
reg_tmp0.x = dot_s(reg_tmp15, reg_tmp1);
reg_tmp0.y = dot_s(reg_tmp15, reg_tmp2);
reg_tmp0.z = dot_s(reg_tmp15, reg_tmp3);
return false;
}
bool sub_3() {
reg_tmp0.x = dot_s(uniforms.f[16], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[17], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[18], reg_tmp15);
return false;
}
bool sub_4() {
if (bool_regs.x) {
sub_5();
} else {
sub_6();
}
return false;
}
bool sub_5() {
reg_tmp1.x = (-uniforms.f[64].xxxx + reg_tmp0.yyyy).x;
reg_tmp1.x = (reg_tmp1.xxxx + reg_tmp1.xxxx).x;
reg_tmp1.w = (mul_s(uniforms.f[94].yyyy, vs_in_reg1.wwww)).w;
reg_tmp1.x = (uniforms.f[95].wwww + -reg_tmp1.xxxx).x;
reg_tmp1.x = (max_s(uniforms.f[95].xxxx, reg_tmp1.xxxx)).x;
vs_out_attr1.w = (mul_s(reg_tmp1.wwww, reg_tmp1.xxxx)).w;
return false;
}
bool sub_6() {
vs_out_attr1.w = (uniforms.f[95].xxxx).w;
return false;
}
bool sub_7() {
vs_out_attr1.w = (mul_s(uniforms.f[94].yyyy, vs_in_reg1.wwww)).w;
return false;
}
bool sub_8() {
reg_tmp1.x = dot_s(uniforms.f[60], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[61], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[62], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[63], reg_tmp0);
return false;
}
bool sub_9() {
reg_tmp1.x = dot_s(uniforms.f[0], reg_tmp0);
reg_tmp1.y = dot_s(uniforms.f[1], reg_tmp0);
reg_tmp1.z = dot_s(uniforms.f[2], reg_tmp0);
reg_tmp1.w = dot_s(uniforms.f[3], reg_tmp0);
return false;
}
bool sub_10() {
reg_tmp1.x = (-reg_tmp1.wwww).x;
return false;
}
bool sub_11() {
reg_tmp12.x = dot_s(uniforms.f[56], reg_tmp0);
reg_tmp12.y = dot_s(uniforms.f[57], reg_tmp0);
reg_tmp12.zw = vec2(dot_s(uniforms.f[59], reg_tmp0));
return false;
}
bool sub_12() {
if (uniforms.b[4]) {
sub_13();
} else {
sub_15();
}
return false;
}
bool sub_13() {
uint jmp_to = 102u;
while (true) {
switch (jmp_to) {
case 102u:
bool_regs = equal(uniforms.f[95].xx, vs_in_reg3.xy);
reg_tmp14 = mul_s(uniforms.f[94].xxxw, vs_in_reg5);
if (bool_regs.x) {
jmp_to = 123u; break;
}
addr_regs.x = (ivec2(vs_in_reg4.xx)).x;
reg_tmp13.w = (mul_s(uniforms.f[94].xxxx, vs_in_reg3.xxxx)).w;
reg_tmp15 = uniforms.f[95].xxxw;
sub_14();
if (bool_regs.y) {
jmp_to = 126u; break;
}
addr_regs.x = (ivec2(vs_in_reg4.yy)).x;
reg_tmp13.w = (mul_s(uniforms.f[94].xxxx, vs_in_reg3.yyyy)).w;
sub_14();
bool_regs = equal(uniforms.f[95].xx, vs_in_reg3.zw);
if (bool_regs.x) {
jmp_to = 126u; break;
}
addr_regs.x = (ivec2(vs_in_reg4.zz)).x;
reg_tmp13.w = (mul_s(uniforms.f[94].xxxx, vs_in_reg3.zzzz)).w;
sub_14();
if (bool_regs.y) {
jmp_to = 126u; break;
}
addr_regs.x = (ivec2(vs_in_reg4.ww)).x;
reg_tmp13.w = (mul_s(uniforms.f[94].xxxx, vs_in_reg3.wwww)).w;
sub_14();
if (!bool_regs.y) {
jmp_to = 126u; break;
}
case 123u:
reg_tmp15.x = dot_3(uniforms.f[16].xyz, reg_tmp14.xyz);
reg_tmp15.y = dot_3(uniforms.f[17].xyz, reg_tmp14.xyz);
reg_tmp15.z = dot_3(uniforms.f[18].xyz, reg_tmp14.xyz);
case 126u:
reg_tmp15.w = dot_3(reg_tmp15.xyz, reg_tmp15.xyz);
reg_tmp15.w = rsq_s(reg_tmp15.w);
reg_tmp3.xyz = (mul_s(reg_tmp15, reg_tmp15.wwww)).xyz;
reg_tmp3.x = dot_3(uniforms.f[8].xyz, reg_tmp3.xyz);
reg_tmp3.y = dot_3(uniforms.f[9].xyz, reg_tmp3.xyz);
reg_tmp3.z = dot_3(uniforms.f[10].xyz, reg_tmp3.xyz);
reg_tmp1.xy = (uniforms.f[95].zzzz).xy;
reg_tmp1.zw = (uniforms.f[95].xxxx).zw;
reg_tmp3 = fma_s(reg_tmp3, reg_tmp1, reg_tmp1);
reg_tmp12.xy = (uniforms.f[19] + reg_tmp3).xy;
reg_tmp12.zw = (uniforms.f[95].wwww).zw;
default: return false;
}
}
return false;
}
bool sub_15() {
reg_tmp0 = mul_s(uniforms.f[95].yyww, vs_in_reg2);
reg_tmp12 = uniforms.f[19] + reg_tmp0;
return false;
}
// reference: 3F24DEB87F4B97AE, 394E558A7A83C1CC
// shader: 8B30, 84602D98F475DF4A
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
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
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
// reference: FDAC838805510368, 84602D98F475DF4A
// program: 394E558A7A83C1CC, 09CF57C48520A714, 84602D98F475DF4A
// shader: 8B30, FFB92148B837BBF2
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
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9318151A14A819AD, FFB92148B837BBF2
// program: 5DFCA9186B211A41, 8F66AB8FEF098E16, FFB92148B837BBF2
// shader: 8B30, FD8459CCFCF7E791
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
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
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
// reference: 349E2197376FA9A8, FD8459CCFCF7E791
// program: 5DFCA9186B211A41, 8F66AB8FEF098E16, FD8459CCFCF7E791
// shader: 8B30, BE637B4A70B2E249
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
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
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
// reference: 00310758E5C5FE1B, BE637B4A70B2E249
// program: 394E558A7A83C1CC, 09CF57C48520A714, BE637B4A70B2E249
// shader: 8B31, F03D18F133364CD9

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

bool sub_3();
bool sub_4();
bool sub_5();
bool sub_0();
bool sub_1();
bool sub_2();
bool sub_6();
bool sub_7();
bool sub_8();

bool exec_shader() {
sub_0();
return true;
}

bool sub_3() {
reg_tmp2.x = dot_s(uniforms.f[16], reg_tmp14);
reg_tmp2.y = dot_s(uniforms.f[17], reg_tmp14);
reg_tmp2.z = dot_s(uniforms.f[18], reg_tmp14);
reg_tmp2.w = (uniforms.f[95].wwww).w;
if (uniforms.b[0]) {
sub_4();
} else {
sub_5();
}
return false;
}
bool sub_4() {
reg_tmp15.x = dot_s(uniforms.f[12], reg_tmp2);
reg_tmp15.y = dot_s(uniforms.f[13], reg_tmp2);
reg_tmp15.z = dot_s(uniforms.f[14], reg_tmp2);
reg_tmp15.w = dot_s(uniforms.f[15], reg_tmp2);
return false;
}
bool sub_5() {
reg_tmp15.x = dot_s(uniforms.f[0], reg_tmp2);
reg_tmp15.y = dot_s(uniforms.f[1], reg_tmp2);
reg_tmp15.z = dot_s(uniforms.f[2], reg_tmp2);
reg_tmp15.w = dot_s(uniforms.f[3], reg_tmp2);
return false;
}
bool sub_0() {
if (uniforms.b[1]) {
sub_1();
} else {
sub_2();
}
reg_tmp1.xy = (mul_s(uniforms.f[19].zwww, reg_tmp1.xyyy)).xy;
sub_3();
reg_tmp2.w = (uniforms.f[20].wwww).w;
bool_regs.x = uniforms.f[95].xxxx.x <= reg_tmp2.wwww.x;
bool_regs.y = uniforms.f[95].xxxx.y < reg_tmp2.wwww.y;
if (bool_regs.x) {
sub_6();
}
vs_out_attr0 = reg_tmp15;
reg_tmp1.y = (uniforms.f[95].wwww + -reg_tmp1.yyyy).y;
vs_out_attr1 = mul_s(uniforms.f[94].yyyy, vs_in_reg1);
vs_out_attr2 = uniforms.f[19] + reg_tmp1;
return true;
}
bool sub_1() {
reg_tmp1.xy = (mul_s(uniforms.f[94].wwww, vs_in_reg2)).xy;
reg_tmp14.xyz = (mul_s(uniforms.f[94].zzzz, vs_in_reg0)).xyz;
reg_tmp14.w = (uniforms.f[95].wwww).w;
return false;
}
bool sub_2() {
reg_tmp1.xy = (vs_in_reg2).xy;
reg_tmp14 = vs_in_reg0;
return false;
}
bool sub_6() {
if (bool_regs.y) {
sub_7();
} else {
sub_8();
}
return false;
}
bool sub_7() {
reg_tmp0.w = rcp_s(reg_tmp15.w);
reg_tmp0.w = (mul_s(uniforms.f[20].wwww, reg_tmp0.wwww)).w;
reg_tmp15 = mul_s(reg_tmp15, reg_tmp0.wwww);
return false;
}
bool sub_8() {
reg_tmp15.z = (uniforms.f[95].xxxx).z;
return false;
}
// reference: 115F4BB91429F3B4, F03D18F133364CD9
// shader: 8B30, 1F1F5184C25285F0
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
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FDAC8388A3344C27, 1F1F5184C25285F0
// program: F03D18F133364CD9, 8F66AB8FEF098E16, 1F1F5184C25285F0
// program: 394E558A7A83C1CC, 09CF57C48520A714, 1F1F5184595C7746
// shader: 8B30, C0ADBAEC991154A9
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
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((texcolor1.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor1.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 2.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3 * 2.0, alpha_output_3 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rrr) + (combiner_buffer.rgb) * (vec3(1) - (const_color[4].rrr)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].r) + (combiner_buffer.a) * (1.0 - (const_color[4].r)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
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
// reference: 87A5C7C916BA7951, C0ADBAEC991154A9
// program: 394E558A7A83C1CC, 09CF57C48520A714, C0ADBAEC991154A9
// shader: 8B30, 76D5737434E5F2C2
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
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
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
// reference: 030DC4B6376FA9A8, 76D5737434E5F2C2
// program: 394E558A7A83C1CC, 09CF57C48520A714, 76D5737434E5F2C2
// shader: 8B30, 263EBB45424C49D5
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
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 030DC4B6FF8EA5DF, 263EBB45424C49D5
// program: 5DFCA9186B211A41, 8F66AB8FEF098E16, 263EBB45424C49D5
// shader: 8B30, 49E2CDD9D523D1C0
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
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 030DC4B60DA93948, 49E2CDD9D523D1C0
// program: F03D18F133364CD9, 8F66AB8FEF098E16, 49E2CDD9D523D1C0

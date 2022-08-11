// shader: 8DD9, 219384019281D7FD

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

out vec4 primary_color;
out vec2 texcoord0;
out vec2 texcoord1;
out vec2 texcoord2;
out float texcoord0_w;
out vec4 normquat;
out vec3 view;

#define NUM_TEV_STAGES 6
layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

in vec4 vs_out_attr0[];
in vec4 vs_out_attr1[];
in vec4 vs_out_attr2[];
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
    primary_color = min(abs(vtx_color), vec4(1.0));

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
// reference: 7B07DA3E334A19B0, 219384019281D7FD
// shader: 8B31, 325E0C369150DE35

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout (std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location = 0) in vec4 vs_in_reg0;
layout(location = 1) in vec4 vs_in_reg1;
layout(location = 2) in vec4 vs_in_reg2;

out vec4 vs_out_attr0;
out vec4 vs_out_attr1;
out vec4 vs_out_attr2;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    exec_shader();
}
bvec2 conditional_code = bvec2(false);
ivec3 address_registers = ivec3(0);
vec4 reg_tmp0 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp1 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp2 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp3 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp4 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp5 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp6 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp7 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp8 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp9 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp10 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp11 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp12 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp13 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp14 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp15 = vec4(0.0, 0.0, 0.0, 1.0);

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
    reg_tmp0.x = dot_s(uniforms.f[0], vs_in_reg0);
    reg_tmp0.y = dot_s(uniforms.f[1], vs_in_reg0);
    reg_tmp0.z = dot_s(uniforms.f[2], vs_in_reg0);
    reg_tmp0.w = dot_s(uniforms.f[3], vs_in_reg0);
    if (uniforms.b[0]) {
        sub_1();
    } else {
        sub_5();
    }
    vs_out_attr1 = mul_s(uniforms.f[47].wwww, vs_in_reg1);
    vs_out_attr2 = vs_in_reg2;
    return true;
}
bool sub_1() {
    if (uniforms.b[1]) {
        sub_2();
    } else {
        sub_3();
    }
    reg_tmp1.y = (mul_s(uniforms.f[5].xxxx, reg_tmp15.yyyy)).y;
    reg_tmp1.y = (uniforms.f[5].yyyy + reg_tmp1.yyyy).y;
    if (uniforms.b[2]) {
        sub_4();
    }
    reg_tmp0.y = (reg_tmp0.yyyy + reg_tmp1.yyyy).y;
    vs_out_attr0 = mul_s(uniforms.f[4], reg_tmp0);
    return false;
}
bool sub_2() {
    reg_tmp15 = vec4(rcp_s(vs_in_reg0.z));
    return false;
}
bool sub_3() {
    reg_tmp15 = vec4(rcp_s(reg_tmp0.w));
    reg_tmp0.zw = (mul_s(reg_tmp0, reg_tmp15)).zw;
    return false;
}
bool sub_4() {
    reg_tmp0.xy = (uniforms.f[47].zzzz + reg_tmp0).xy;
    reg_tmp1.y = (floor(reg_tmp1.yyyy)).y;
    reg_tmp0.xy = (floor(reg_tmp0)).xy;
    return false;
}
bool sub_5() {
    if (uniforms.b[2]) {
        sub_6();
    }
    vs_out_attr0 = mul_s(uniforms.f[4], reg_tmp0);
    return false;
}
bool sub_6() {
    reg_tmp0.xy = (uniforms.f[47].zzzz + reg_tmp0).xy;
    reg_tmp0.xy = (floor(reg_tmp0)).xy;
    return false;
}
// reference: 1C94529D46648756, 325E0C369150DE35
// shader: 8B30, 728E1E63C264CB58
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24
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
layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

vec2 byteround(vec2 x) {
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

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5949BCE7D40ACA34, 728E1E63C264CB58
// program: 325E0C369150DE35, 219384019281D7FD, 728E1E63C264CB58
// shader: 8DD9, 7874227BFE44E47C

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

out vec4 primary_color;
out vec2 texcoord0;
out vec2 texcoord1;
out vec2 texcoord2;
out float texcoord0_w;
out vec4 normquat;
out vec3 view;

#define NUM_TEV_STAGES 6
layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

in vec4 vs_out_attr0[];
in vec4 vs_out_attr1[];
in vec4 vs_out_attr2[];
in vec4 vs_out_attr3[];
in vec4 vs_out_attr4[];
in vec4 vs_out_attr5[];
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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z);
    texcoord2 = vec2(vtx.attributes[5].z, vtx.attributes[5].w);

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
// reference: 6B326F6AE5AB004C, 7874227BFE44E47C
// shader: 8B31, 308CE2DAD6FAEB6D

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout (std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location = 0) in vec4 vs_in_reg0;
layout(location = 1) in vec4 vs_in_reg1;
layout(location = 2) in vec4 vs_in_reg2;
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;
layout(location = 7) in vec4 vs_in_reg7;

out vec4 vs_out_attr0;
out vec4 vs_out_attr1;
out vec4 vs_out_attr2;
out vec4 vs_out_attr3;
out vec4 vs_out_attr4;
out vec4 vs_out_attr5;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr4 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr5 = vec4(0.0, 0.0, 0.0, 1.0);
    exec_shader();
}
bvec2 conditional_code = bvec2(false);
ivec3 address_registers = ivec3(0);
vec4 reg_tmp0 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp1 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp2 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp3 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp4 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp5 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp6 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp7 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp8 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp9 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp10 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp11 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp12 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp13 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp14 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp15 = vec4(0.0, 0.0, 0.0, 1.0);

bool sub_25();
bool sub_19();
bool sub_4();
bool sub_6();
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
bool sub_0();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_5();
bool sub_17();
bool sub_18();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_25() {
    reg_tmp15 = floor(vs_in_reg4);
    reg_tmp15 = vs_in_reg4 + -reg_tmp15;
    reg_tmp15 = reg_tmp15 + reg_tmp15;
    address_registers.xy = ivec2(vs_in_reg4.xy);
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.x], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.x], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.x], vs_in_reg0);
    reg_tmp0.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp15.xxxx)).xyz;
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.y], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.y], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.y], vs_in_reg0);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.yyyy, reg_tmp0.xyzz)).xyz;
    address_registers.xy = ivec2(vs_in_reg4.zw);
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.x], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.x], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.x], vs_in_reg0);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.zzzz, reg_tmp0.xyzz)).xyz;
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.y], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.y], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.y], vs_in_reg0);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.wwww, reg_tmp0.xyzz)).xyz;
    return false;
}
bool sub_19() {
    reg_tmp15 = floor(vs_in_reg4);
    reg_tmp15 = vs_in_reg4 + -reg_tmp15;
    reg_tmp15 = reg_tmp15 + reg_tmp15;
    address_registers.xy = ivec2(vs_in_reg4.xy);
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.x], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.x], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.x], vs_in_reg0);
    reg_tmp13.x = dot_3(uniforms.f[33 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp13.y = dot_3(uniforms.f[34 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp13.z = dot_3(uniforms.f[35 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp0.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp15.xxxx)).xyz;
    reg_tmp1.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp15.xxxx)).xyz;
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.y], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.y], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.y], vs_in_reg0);
    reg_tmp13.x = dot_3(uniforms.f[33 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp13.y = dot_3(uniforms.f[34 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp13.z = dot_3(uniforms.f[35 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.yyyy, reg_tmp0.xyzz)).xyz;
    reg_tmp1.xyz = (fma_s(reg_tmp13.xyzz, reg_tmp15.yyyy, reg_tmp1.xyzz)).xyz;
    address_registers.xy = ivec2(vs_in_reg4.zw);
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.x], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.x], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.x], vs_in_reg0);
    reg_tmp13.x = dot_3(uniforms.f[33 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp13.y = dot_3(uniforms.f[34 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp13.z = dot_3(uniforms.f[35 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.zzzz, reg_tmp0.xyzz)).xyz;
    reg_tmp1.xyz = (fma_s(reg_tmp13.xyzz, reg_tmp15.zzzz, reg_tmp1.xyzz)).xyz;
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.y], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.y], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.y], vs_in_reg0);
    reg_tmp13.x = dot_3(uniforms.f[33 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp13.y = dot_3(uniforms.f[34 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp13.z = dot_3(uniforms.f[35 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp1.xyz = (fma_s(reg_tmp13.xyzz, reg_tmp15.wwww, reg_tmp1.xyzz)).xyz;
    return false;
}
bool sub_4() {
    reg_tmp15 = floor(vs_in_reg4);
    reg_tmp15 = vs_in_reg4 + -reg_tmp15;
    reg_tmp15 = reg_tmp15 + reg_tmp15;
    address_registers.xy = ivec2(vs_in_reg4.xy);
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.x], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.x], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.x], vs_in_reg0);
    reg_tmp13.x = dot_3(uniforms.f[33 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp13.y = dot_3(uniforms.f[34 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp13.z = dot_3(uniforms.f[35 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp12.x = dot_3(uniforms.f[33 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp12.y = dot_3(uniforms.f[34 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp12.z = dot_3(uniforms.f[35 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp0.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp15.xxxx)).xyz;
    reg_tmp1.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp15.xxxx)).xyz;
    reg_tmp2.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp15.xxxx)).xyz;
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.y], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.y], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.y], vs_in_reg0);
    reg_tmp13.x = dot_3(uniforms.f[33 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp13.y = dot_3(uniforms.f[34 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp13.z = dot_3(uniforms.f[35 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp12.x = dot_3(uniforms.f[33 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp12.y = dot_3(uniforms.f[34 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp12.z = dot_3(uniforms.f[35 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.yyyy, reg_tmp0.xyzz)).xyz;
    reg_tmp1.xyz = (fma_s(reg_tmp13.xyzz, reg_tmp15.yyyy, reg_tmp1.xyzz)).xyz;
    reg_tmp2.xyz = (fma_s(reg_tmp12.xyzz, reg_tmp15.yyyy, reg_tmp2.xyzz)).xyz;
    address_registers.xy = ivec2(vs_in_reg4.zw);
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.x], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.x], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.x], vs_in_reg0);
    reg_tmp13.x = dot_3(uniforms.f[33 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp13.y = dot_3(uniforms.f[34 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp13.z = dot_3(uniforms.f[35 + address_registers.x].xyz, vs_in_reg1.xyz);
    reg_tmp12.x = dot_3(uniforms.f[33 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp12.y = dot_3(uniforms.f[34 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp12.z = dot_3(uniforms.f[35 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.zzzz, reg_tmp0.xyzz)).xyz;
    reg_tmp1.xyz = (fma_s(reg_tmp13.xyzz, reg_tmp15.zzzz, reg_tmp1.xyzz)).xyz;
    reg_tmp2.xyz = (fma_s(reg_tmp12.xyzz, reg_tmp15.zzzz, reg_tmp2.xyzz)).xyz;
    reg_tmp14.x = dot_s(uniforms.f[33 + address_registers.y], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[34 + address_registers.y], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[35 + address_registers.y], vs_in_reg0);
    reg_tmp13.x = dot_3(uniforms.f[33 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp13.y = dot_3(uniforms.f[34 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp13.z = dot_3(uniforms.f[35 + address_registers.y].xyz, vs_in_reg1.xyz);
    reg_tmp12.x = dot_3(uniforms.f[33 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp12.y = dot_3(uniforms.f[34 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp12.z = dot_3(uniforms.f[35 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp0.xyz = (fma_s(reg_tmp14.xyzz, reg_tmp15.wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp1.xyz = (fma_s(reg_tmp13.xyzz, reg_tmp15.wwww, reg_tmp1.xyzz)).xyz;
    reg_tmp2.xyz = (fma_s(reg_tmp12.xyzz, reg_tmp15.wwww, reg_tmp2.xyzz)).xyz;
    return false;
}
bool sub_6() {
    reg_tmp5 = mul_s(reg_tmp1.yzxx, reg_tmp2.zxyy);
    reg_tmp5 = fma_s(-reg_tmp2.yzxx, reg_tmp1.zxyy, reg_tmp5);
    reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
    reg_tmp5.w = rsq_s(reg_tmp5.w);
    reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
    reg_tmp6.w = (reg_tmp1.zzzz + reg_tmp5.yyyy).w;
    reg_tmp2 = mul_s(reg_tmp5.yzxx, reg_tmp1.zxyy);
    reg_tmp2 = fma_s(-reg_tmp1.yzxx, reg_tmp5.zxyy, reg_tmp2);
    reg_tmp6.w = (reg_tmp2.xxxx + reg_tmp6.wwww).w;
    reg_tmp2.w = (reg_tmp5.zzzz).w;
    reg_tmp5.z = (reg_tmp2.xxxx).z;
    reg_tmp6.w = (uniforms.f[32].xxxx + reg_tmp6.wwww).w;
    reg_tmp1.w = (reg_tmp5.xxxx).w;
    reg_tmp5.x = (reg_tmp1.zzzz).x;
    conditional_code = lessThan(uniforms.f[31].xx, reg_tmp6.ww);
    reg_tmp6.x = (uniforms.f[32].xxxx).x;
    reg_tmp6.y = (-uniforms.f[32].xxxx).y;
    if (!conditional_code.x) {
        sub_7();
    } else {
        sub_16();
    }
    return false;
}
bool sub_7() {
    conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
    if (conditional_code.x) {
        sub_8();
    } else {
        sub_13();
    }
    reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
    vs_out_attr2 = mul_s(reg_tmp8, reg_tmp6);
    return false;
}
bool sub_8() {
    if (conditional_code.y) {
        sub_9();
    } else {
        sub_10();
    }
    reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
    return false;
}
bool sub_9() {
    reg_tmp8 = mul_s(reg_tmp2.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[32].xxxx + -reg_tmp5.yyyy).x;
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp1.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_10() {
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    reg_tmp8 = mul_s(reg_tmp2.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[32].xxxx + -reg_tmp5.yyyy).x;
    if (conditional_code.x) {
        sub_11();
    } else {
        sub_12();
    }
    return false;
}
bool sub_11() {
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp1.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_12() {
    reg_tmp8 = mul_s(reg_tmp2.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[32].xxxx + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp1.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_13() {
    if (conditional_code.y) {
        sub_14();
    } else {
        sub_15();
    }
    reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
    return false;
}
bool sub_14() {
    reg_tmp8 = mul_s(reg_tmp2.yywz, reg_tmp6.xxxy);
    reg_tmp8.y = (uniforms.f[32].xxxx + -reg_tmp5.zzzz).y;
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp1.wwyx).xzw;
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_15() {
    reg_tmp8 = mul_s(reg_tmp2.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[32].xxxx + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp1.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_16() {
    reg_tmp7.xz = (reg_tmp2.wwyy + -reg_tmp1.yyww).xz;
    reg_tmp7.y = (reg_tmp1.xxxx + -reg_tmp2.zzzz).y;
    reg_tmp7.w = (reg_tmp6.wwww).w;
    reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
    reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
    vs_out_attr2 = mul_s(reg_tmp7, reg_tmp6);
    return false;
}
bool sub_0() {
    if (uniforms.b[1]) {
        sub_1();
    } else {
        sub_23();
    }
    reg_tmp15 = mul_s(uniforms.f[12].xxxx, vs_in_reg3);
    vs_out_attr1 = uniforms.f[12].yyyy + reg_tmp15;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp0);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp0);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp0);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp0);
    reg_tmp11 = uniforms.f[32].yyyy;
    if (uniforms.b[3]) {
        sub_27();
    }
    if (uniforms.b[4]) {
        sub_28();
    }
    vs_out_attr0 = reg_tmp10 + reg_tmp11;
    reg_tmp13 = mul_s(uniforms.f[16].xxxx, vs_in_reg5);
    reg_tmp14 = mul_s(uniforms.f[17].xxxx, vs_in_reg5);
    reg_tmp15 = mul_s(uniforms.f[18].xxxx, vs_in_reg5);
    reg_tmp13 = fma_s(vs_in_reg6, uniforms.f[16].yyyy, reg_tmp13);
    reg_tmp14 = fma_s(vs_in_reg6, uniforms.f[17].yyyy, reg_tmp14);
    reg_tmp15 = fma_s(vs_in_reg6, uniforms.f[18].yyyy, reg_tmp15);
    reg_tmp13 = fma_s(vs_in_reg7, uniforms.f[16].zzzz, reg_tmp13);
    reg_tmp14 = fma_s(vs_in_reg7, uniforms.f[17].zzzz, reg_tmp14);
    reg_tmp15 = fma_s(vs_in_reg7, uniforms.f[18].zzzz, reg_tmp15);
    vs_out_attr4 = uniforms.f[12].zwzw + reg_tmp13;
    vs_out_attr5.xy = (uniforms.f[13] + reg_tmp14.xyxy).xy;
    if (uniforms.b[5]) {
        sub_29();
    } else {
        sub_30();
    }
    return true;
}
bool sub_1() {
    if (uniforms.b[2]) {
        sub_2();
    } else {
        sub_17();
    }
    vs_out_attr3 = uniforms.f[4] + -reg_tmp0;
    return false;
}
bool sub_2() {
    if (uniforms.b[0]) {
        sub_3();
    } else {
        sub_5();
    }
    reg_tmp15.x = dot_3(reg_tmp1.xyz, reg_tmp1.xyz);
    reg_tmp14.x = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
    reg_tmp15.x = rsq_s(reg_tmp15.x);
    reg_tmp14.x = rsq_s(reg_tmp14.x);
    reg_tmp1.xyz = (mul_s(reg_tmp1.xyzz, reg_tmp15.xxxx)).xyz;
    reg_tmp2.xyz = (mul_s(reg_tmp2.xyzz, reg_tmp14.xxxx)).xyz;
    {
        sub_6();
    }
    return false;
}
bool sub_3() {
    {
        sub_4();
    }
    reg_tmp0.w = (vs_in_reg0).w;
    return false;
}
bool sub_5() {
    reg_tmp2.x = dot_3(uniforms.f[33].xyz, vs_in_reg2.xyz);
    reg_tmp2.y = dot_3(uniforms.f[34].xyz, vs_in_reg2.xyz);
    reg_tmp2.z = dot_3(uniforms.f[35].xyz, vs_in_reg2.xyz);
    reg_tmp1.x = dot_3(uniforms.f[33].xyz, vs_in_reg1.xyz);
    reg_tmp1.y = dot_3(uniforms.f[34].xyz, vs_in_reg1.xyz);
    reg_tmp1.z = dot_3(uniforms.f[35].xyz, vs_in_reg1.xyz);
    reg_tmp0.x = dot_s(uniforms.f[33], vs_in_reg0);
    reg_tmp0.y = dot_s(uniforms.f[34], vs_in_reg0);
    reg_tmp0.z = dot_s(uniforms.f[35], vs_in_reg0);
    reg_tmp0.w = (vs_in_reg0).w;
    return false;
}
bool sub_17() {
    if (uniforms.b[0]) {
        sub_18();
    } else {
        sub_20();
    }
    reg_tmp15.x = dot_3(reg_tmp1.xyz, reg_tmp1.xyz);
    reg_tmp15.x = rsq_s(reg_tmp15.x);
    reg_tmp1.xyz = (mul_s(reg_tmp1.xyzz, reg_tmp15.xxxx)).xyz;
    conditional_code = equal(-uniforms.f[32].xx, reg_tmp1.zz);
    reg_tmp14 = uniforms.f[32].xxxx + reg_tmp1.zzzz;
    reg_tmp14 = mul_s(uniforms.f[32].wwww, reg_tmp14);
    reg_tmp14 = vec4(rsq_s(reg_tmp14.x));
    reg_tmp15.xyz = (mul_s(uniforms.f[32].wwww, reg_tmp1.xyzz)).xyz;
    if (!conditional_code.x) {
        sub_21();
    } else {
        sub_22();
    }
    vs_out_attr2.w = (uniforms.f[32].yyyy).w;
    return false;
}
bool sub_18() {
    {
        sub_19();
    }
    reg_tmp0.w = (vs_in_reg0).w;
    return false;
}
bool sub_20() {
    reg_tmp1.x = dot_3(uniforms.f[33].xyz, vs_in_reg1.xyz);
    reg_tmp1.y = dot_3(uniforms.f[34].xyz, vs_in_reg1.xyz);
    reg_tmp1.z = dot_3(uniforms.f[35].xyz, vs_in_reg1.xyz);
    reg_tmp0.x = dot_s(uniforms.f[33], vs_in_reg0);
    reg_tmp0.y = dot_s(uniforms.f[34], vs_in_reg0);
    reg_tmp0.z = dot_s(uniforms.f[35], vs_in_reg0);
    reg_tmp0.w = (vs_in_reg0).w;
    return false;
}
bool sub_21() {
    vs_out_attr2.z = rcp_s(reg_tmp14.x);
    vs_out_attr2.xy = (mul_s(reg_tmp15.xyyy, reg_tmp14)).xy;
    return false;
}
bool sub_22() {
    vs_out_attr2.xyz = (uniforms.f[32].xyyy).xyz;
    return false;
}
bool sub_23() {
    if (uniforms.b[0]) {
        sub_24();
    } else {
        sub_26();
    }
    vs_out_attr2 = uniforms.f[32].xyyy;
    vs_out_attr3 = uniforms.f[32].yyxx;
    return false;
}
bool sub_24() {
    {
        sub_25();
    }
    reg_tmp0.w = (vs_in_reg0).w;
    return false;
}
bool sub_26() {
    reg_tmp0.x = dot_s(uniforms.f[33], vs_in_reg0);
    reg_tmp0.y = dot_s(uniforms.f[34], vs_in_reg0);
    reg_tmp0.z = dot_s(uniforms.f[35], vs_in_reg0);
    reg_tmp0.w = (vs_in_reg0).w;
    return false;
}
bool sub_27() {
    reg_tmp15.x = (uniforms.f[7].zzzz).x;
    reg_tmp15.y = rcp_s(reg_tmp10.w);
    reg_tmp15.x = (mul_s(uniforms.f[7].wwww, reg_tmp15.xxxx)).x;
    reg_tmp12 = vec4(greaterThanEqual(-reg_tmp10.wwww, uniforms.f[7].wwww));
    reg_tmp10.w = (uniforms.f[32].xxxx).w;
    reg_tmp11.y = (fma_s(reg_tmp15.yyyy, reg_tmp15.xxxx, uniforms.f[7].zzzz)).y;
    return false;
}
bool sub_28() {
    reg_tmp10.xy = (floor(reg_tmp10)).xy;
    reg_tmp10.xy = (mul_s(uniforms.f[7].xyyy, reg_tmp10)).xy;
    reg_tmp11.xy = (floor(reg_tmp11)).xy;
    reg_tmp11.xy = (mul_s(uniforms.f[7].xyyy, reg_tmp11)).xy;
    return false;
}
bool sub_29() {
    reg_tmp10.xy = (fma_s(reg_tmp11, reg_tmp12, reg_tmp10)).xy;
    reg_tmp10.xy = (mul_s(uniforms.f[8].xyxy, reg_tmp10.xyyy)).xy;
    vs_out_attr5.zw = (uniforms.f[8].zwzw + reg_tmp10.xyxy).zw;
    return false;
}
bool sub_30() {
    vs_out_attr5.zw = (uniforms.f[13] + reg_tmp15.xyxy).zw;
    return false;
}
// reference: B82A6F19250D9724, 308CE2DAD6FAEB6D
// shader: 8B30, 49990ECE39C8D5BE
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24
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
layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

vec2 byteround(vec2 x) {
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

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1.0) - const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 70047AE025C9B0D9, 49990ECE39C8D5BE
// program: 308CE2DAD6FAEB6D, 7874227BFE44E47C, 49990ECE39C8D5BE
// shader: 8B30, F4F7898257145182
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24
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
layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

vec2 byteround(vec2 x) {
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

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1.0) - const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CC1B96C3423E24AF, F4F7898257145182
// program: 308CE2DAD6FAEB6D, 7874227BFE44E47C, F4F7898257145182
// shader: 8B30, 6FB5F97A64AAEA77
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24
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
layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

vec2 byteround(vec2 x) {
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

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1.0) - const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CC1B96C325C9B0D9, 6FB5F97A64AAEA77
// program: 308CE2DAD6FAEB6D, 7874227BFE44E47C, 6FB5F97A64AAEA77
// shader: 8B30, 88261D372F39896F
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24
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
layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

vec2 byteround(vec2 x) {
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

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1.0) - const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 70047AE0423E24AF, 88261D372F39896F
// program: 308CE2DAD6FAEB6D, 7874227BFE44E47C, 88261D372F39896F
// shader: 8B30, D9FB2F0932999E52
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24
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
layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

vec2 byteround(vec2 x) {
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

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5949BCE738BD2137, D9FB2F0932999E52
// program: 325E0C369150DE35, 219384019281D7FD, D9FB2F0932999E52

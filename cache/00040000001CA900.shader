// shader: 8B31, D09770228B1223B9

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

 out vec4 vs_out_attr0;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_0_4096();
bool sub_5_13();
bool sub_13_25();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    // 0: dphi
    reg_tmp13.x = dot(mul_safe(vec4(vs_in_reg0.xyz, 1.0), uniforms.f[0]), vec4(1.0));
    // 1: dphi
    reg_tmp13.y = dot(mul_safe(vec4(vs_in_reg0.xyz, 1.0), uniforms.f[1]), vec4(1.0));
    // 2: dphi
    reg_tmp13.z = dot(mul_safe(vec4(vs_in_reg0.xyz, 1.0), uniforms.f[2]), vec4(1.0));
    // 3: mov
    reg_tmp13.w = (vs_in_reg0.wwww).w;
    // 4: ifu
    if (uniforms.b[3]) {
        sub_5_13();
    } else {
        sub_13_25();
    }
    // 25: end
    return true;
}
bool sub_5_13() {
    // 5: dphi
    reg_tmp12.x = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[90]), vec4(1.0));
    // 6: dphi
    reg_tmp12.y = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[91]), vec4(1.0));
    // 7: dphi
    reg_tmp12.z = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[92]), vec4(1.0));
    // 8: mov
    reg_tmp12.w = (reg_tmp13.wwww).w;
    // 9: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp12), vec4(1.0));
    // 10: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp12), vec4(1.0));
    // 11: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp12), vec4(1.0));
    // 12: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp12), vec4(1.0));
    return false;
}
bool sub_13_25() {
    // 13: rcp
    reg_tmp11.x = rcp_safe(reg_tmp13.zzzz.x);
    // 14: add
    reg_tmp10.x = (-uniforms.f[85].yyyy + reg_tmp13.zzzz).x;
    // 15: mul
    reg_tmp9.x = (mul_safe(uniforms.f[85].wwww, reg_tmp11.xxxx)).x;
    // 16: mul
    reg_tmp12.x = (mul_safe(reg_tmp9.xxxx, reg_tmp10.xxxx)).x;
    // 17: mov
    reg_tmp13.z = (-reg_tmp13.zzzz).z;
    // 18: add
    reg_tmp13.x = (reg_tmp13.xxxx + reg_tmp12.xxxx).x;
    // 19: dphi
    reg_tmp12.w = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[89]), vec4(1.0));
    // 20: dphi
    reg_tmp12.z = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[88]), vec4(1.0));
    // 21: dphi
    reg_tmp12.x = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[5]), vec4(1.0));
    // 22: dphi
    reg_tmp12.y = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[6]), vec4(1.0));
    // 23: mov
    vs_out_attr0.zw = (reg_tmp12).zw;
    // 24: mul
    vs_out_attr0.xy = (mul_safe(reg_tmp12.xyyy, reg_tmp12.wwww)).xy;
    return false;
}
// reference: DE29E4B6FCDD239B, D09770228B1223B9
// shader: 8DD9, 0282066502FA87A6

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

 in vec4 vs_out_attr0[];

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
    primary_color = min(abs(vtx_color), vec4(1.0));

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
// reference: B0EECBB083D64E3E, 0282066502FA87A6
// shader: 8B30, 47C12CB4EFB88BF9

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D4F4BEF997156CBA, 47C12CB4EFB88BF9
// program: D09770228B1223B9, 0282066502FA87A6, 47C12CB4EFB88BF9
// shader: 8B31, 8809BF824E86BD32

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
layout(location = 8) in vec4 vs_in_reg8;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;
 out vec4 vs_out_attr4;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr4 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_7_12();
bool sub_12_21();
bool sub_21_34();
bool sub_34_209();
bool sub_40_142();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_142_208();
bool sub_145_155();
bool sub_155_163();
bool sub_164_176();
bool sub_176_201();
bool sub_177_195();
bool sub_195_200();
bool sub_209_226();
bool sub_226_302();
bool sub_265_285();
bool sub_266_271();
bool sub_271_284();
bool sub_275_278();
bool sub_278_283();
bool sub_285_298();
bool sub_286_291();
bool sub_291_297();
bool sub_302_312();
bool sub_307_311();
bool sub_308_309();
bool sub_312_339();
bool sub_314_319();
bool sub_319_338();
bool sub_322_328();
bool sub_328_337();
bool sub_329_333();
bool sub_333_336();
bool sub_339_347();
bool sub_341_342();
bool sub_342_346();
bool sub_343_344();
bool sub_344_345();
bool sub_347_354();
bool sub_354_358();
bool sub_404_4096();

bool exec_shader() {
    sub_404_4096();
    return true;
}

bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 8: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 9: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 10: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 11: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 13: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 14: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 15: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 16: dp3
    reg_tmp4.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 17: dp3
    reg_tmp4.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 18: dp3
    reg_tmp4.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 19: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 22: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 23: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 24: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 25: dp3
    reg_tmp4.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 26: dp3
    reg_tmp4.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 27: dp3
    reg_tmp4.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 28: dp3
    reg_tmp5.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 29: dp3
    reg_tmp5.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 30: dp3
    reg_tmp5.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 31: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_safe(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_209() {
    // 34: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_142();
    } else {
        sub_142_208();
    }
    // 208: nop
    return false;
}
bool sub_40_142() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 42: mov
    reg_tmp7 = uniforms.f[93].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[93].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[93].xxxx;
    // 45: mul
    reg_tmp2 = mul_safe(uniforms.f[93].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 136: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 137: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 138: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 139: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 140: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 141: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 64: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 65: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 66: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 68: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 69: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 70: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 72: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 73: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 74: call
    {
        sub_209_226();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 60: callc
    if (conditional_code.y) {
        sub_12_21();
    }
    // 61: nop
    return false;
}
bool sub_76_135() {
    // 76: ifc
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    // 134: nop
    return false;
}
bool sub_77_109() {
    // 77: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 94: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 95: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 96: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 98: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 99: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 100: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 101: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 102: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 103: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 104: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 105: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 106: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 107: call
    {
        sub_226_302();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 126: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 127: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 128: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 130: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 131: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 132: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_142_208() {
    // 142: mov
    reg_tmp0 = uniforms.f[7];
    // 143: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 144: ifu
    if (uniforms.b[2]) {
        sub_145_155();
    } else {
        sub_155_163();
    }
    // 163: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_164_176();
    } else {
        sub_176_201();
    }
    // 201: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 202: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 203: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 204: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 205: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 206: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 207: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_145_155() {
    // 145: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 146: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 147: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 148: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 149: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 150: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 151: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 152: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 153: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 154: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_155_163() {
    // 155: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22], reg_tmp15), vec4(1.0));
    // 156: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23], reg_tmp15), vec4(1.0));
    // 157: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24], reg_tmp15), vec4(1.0));
    // 158: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 159: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 160: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 161: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 162: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_164_176() {
    // 164: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 165: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 166: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 167: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 168: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 169: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 170: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 171: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 172: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 173: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 174: call
    {
        sub_209_226();
    }
    // 175: nop
    return false;
}
bool sub_176_201() {
    // 176: ifc
    if (all(conditional_code)) {
        sub_177_195();
    } else {
        sub_195_200();
    }
    // 200: nop
    return false;
}
bool sub_177_195() {
    // 177: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 178: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 179: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 180: dp3
    reg_tmp11.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 181: dp3
    reg_tmp11.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 182: dp3
    reg_tmp11.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 183: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 184: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 185: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 186: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 187: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 188: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 189: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 190: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 191: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 192: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 193: call
    {
        sub_226_302();
    }
    // 194: nop
    return false;
}
bool sub_195_200() {
    // 195: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 196: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 197: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 198: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 199: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_209_226() {
    uint jmp_to = 209u;
    while (true) {
        switch (jmp_to) {
        case 209u: {
            // 209: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 210: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 211: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 212: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 213: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 214: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 215: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 216: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 225u; break; }
            }
            // 217: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 218: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 219: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 220: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 221: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 222: jmpc
            if (conditional_code.x) {
                { jmp_to = 225u; break; }
            }
            // 223: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 224: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 225u: {
            // 225: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_226_302() {
    uint jmp_to = 226u;
    while (true) {
        switch (jmp_to) {
        case 226u: {
            // 226: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 227: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 228: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 229: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 230: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 231: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 232: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 233: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 234: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 235: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 301u; break; }
            }
            // 236: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 237: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 238: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 239: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 240: dp3
            reg_tmp5.w = dot(vec3(mul_safe(reg_tmp5, reg_tmp5)), vec3(1.0));
            // 241: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 242: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 243: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 244: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 245: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 246: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 247: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 248: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 249: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 250: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 251: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 252: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 253: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 254: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 255: jmpc
            if (!conditional_code.x) {
                { jmp_to = 263u; break; }
            }
            // 256: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 257: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 258: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 259: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp7, reg_tmp7), vec4(1.0)));
            // 260: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 261: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 262: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 301u; break; }
            }
        }
        case 263u: {
            // 263: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 264: ifc
            if (conditional_code.x) {
                sub_265_285();
            } else {
                sub_285_298();
            }
            // 298: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp8, reg_tmp8), vec4(1.0)));
            // 299: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 300: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 301u: {
            // 301: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_265_285() {
    // 265: ifc
    if (conditional_code.y) {
        sub_266_271();
    } else {
        sub_271_284();
    }
    // 284: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_266_271() {
    // 266: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 267: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 268: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 269: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 270: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_271_284() {
    // 271: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 272: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 273: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 274: ifc
    if (conditional_code.x) {
        sub_275_278();
    } else {
        sub_278_283();
    }
    // 283: nop
    return false;
}
bool sub_275_278() {
    // 275: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 276: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 277: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_278_283() {
    // 278: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 279: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 280: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 281: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 282: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_285_298() {
    // 285: ifc
    if (conditional_code.y) {
        sub_286_291();
    } else {
        sub_291_297();
    }
    // 297: nop
    return false;
}
bool sub_286_291() {
    // 286: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 287: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 288: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 289: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 290: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_291_297() {
    // 291: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 292: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 293: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 294: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 295: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 296: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_302_312() {
    // 302: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 303: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 304: mov
    reg_tmp9 = uniforms.f[21];
    // 305: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 306: ifc
    if (conditional_code.y) {
        sub_307_311();
    }
    // 311: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_307_311() {
    // 307: ifu
    if (uniforms.b[7]) {
        sub_308_309();
    }
    // 309: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 310: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_308_309() {
    // 308: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_312_339() {
    // 312: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 313: ifu
    if (uniforms.b[9]) {
        sub_314_319();
    } else {
        sub_319_338();
    }
    // 338: nop
    return false;
}
bool sub_314_319() {
    // 314: call
    {
        sub_339_347();
    }
    // 315: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11].xywz, reg_tmp6), vec4(1.0));
    // 316: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12].xywz, reg_tmp6), vec4(1.0));
    // 317: mov
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    // 318: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_319_338() {
    // 319: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 320: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 321: ifc
    if (all(not(conditional_code))) {
        sub_322_328();
    } else {
        sub_328_337();
    }
    // 337: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_322_328() {
    // 322: mov
    reg_tmp6 = reg_tmp10;
    // 323: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11], reg_tmp6), vec4(1.0));
    // 324: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12], reg_tmp6), vec4(1.0));
    // 325: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[13], reg_tmp6), vec4(1.0));
    // 326: mul
    reg_tmp0.xy = (mul_safe(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 327: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_328_337() {
    // 328: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_329_333();
    } else {
        sub_333_336();
    }
    // 336: nop
    return false;
}
bool sub_329_333() {
    // 329: call
    {
        sub_347_354();
    }
    // 330: dp3
    reg_tmp3.x = dot(vec3(mul_safe(uniforms.f[11], reg_tmp6)), vec3(1.0));
    // 331: dp3
    reg_tmp3.y = dot(vec3(mul_safe(uniforms.f[12], reg_tmp6)), vec3(1.0));
    // 332: dp3
    reg_tmp3.z = dot(vec3(mul_safe(uniforms.f[13], reg_tmp6)), vec3(1.0));
    return false;
}
bool sub_333_336() {
    // 333: call
    {
        sub_354_358();
    }
    // 334: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11], reg_tmp6), vec4(1.0));
    // 335: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12], reg_tmp6), vec4(1.0));
    return false;
}
bool sub_339_347() {
    // 339: cmp
    conditional_code = equal(vec2(uniforms.f[93].yzzz), vec2(reg_tmp0.xyyy));
    // 340: ifc
    if (all(not(conditional_code))) {
        sub_341_342();
    } else {
        sub_342_346();
    }
    // 346: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_341_342() {
    // 341: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_342_346() {
    // 342: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_343_344();
    } else {
        sub_344_345();
    }
    // 345: nop
    return false;
}
bool sub_343_344() {
    // 343: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_344_345() {
    // 344: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_347_354() {
    // 347: mov
    reg_tmp2 = -reg_tmp15;
    // 348: dp3
    reg_tmp2.w = dot(vec3(mul_safe(reg_tmp2, reg_tmp2)), vec3(1.0));
    // 349: rsq
    reg_tmp2.w = rsq_safe(reg_tmp2.wwww.x);
    // 350: mul
    reg_tmp2 = mul_safe(reg_tmp2, reg_tmp2.wwww);
    // 351: dp3
    reg_tmp1 = vec4(dot(vec3(mul_safe(reg_tmp2, reg_tmp14)), vec3(1.0)));
    // 352: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 353: mad
    reg_tmp6 = fma_safe(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_354_358() {
    // 354: mov
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    // 355: mov
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    // 356: mad
    reg_tmp6 = fma_safe(reg_tmp14, reg_tmp1, reg_tmp1);
    // 357: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_404_4096() {
    // 404: call
    {
        sub_34_209();
    }
    // 405: call
    {
        sub_302_312();
    }
    // 406: call
    {
        sub_312_339();
    }
    // 407: end
    return true;
}
// reference: 7EA9C1D92DE46B42, 8809BF824E86BD32
// shader: 8DD9, 6CF3F3B70E23AA85

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

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];
 in vec4 vs_out_attr4[];

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
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(0.0, 0.0);

    texcoord0_w = vtx.attributes[4].z;
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
// reference: CD8210802464D9AE, 6CF3F3B70E23AA85
// shader: 8B30, 05D8F71C78F19800

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g) - (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 1.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A8761BA4B56473C7, 05D8F71C78F19800
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 05D8F71C78F19800
// reference: F5491676F65DCA05, 8809BF824E86BD32
// shader: 8B30, 4C2527A078F19800

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) - (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 1.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CF134005B56473C7, 4C2527A078F19800
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 4C2527A078F19800
// reference: 3D62A9C8A1ADE3A3, D09770228B1223B9
// reference: B6827E677A1442E4, D09770228B1223B9
// shader: 8B30, 077C1C336144D3DF

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 62769A25330A4935, 077C1C336144D3DF
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 077C1C336144D3DF
// shader: 8B30, B038D2DA51D280D0

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B7261CD4AAE3F59A, B038D2DA51D280D0
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, B038D2DA51D280D0
// shader: 8B30, 0F6657B9E57FE0A0

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1.0) - (texcolor0.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp(min((const_color[2].rgb) + (rounded_primary_color.rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 754186C8CECF130E, 0F6657B9E57FE0A0
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 0F6657B9E57FE0A0
// shader: 8B31, 4678A4FD452DD0E4

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
layout(location = 8) in vec4 vs_in_reg8;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;
 out vec4 vs_out_attr4;
 out vec4 vs_out_attr5;
 out vec4 vs_out_attr6;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr4 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr5 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr6 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_0_4096();
bool sub_7_12();
bool sub_12_21();
bool sub_21_34();
bool sub_34_209();
bool sub_40_142();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_142_208();
bool sub_145_155();
bool sub_155_163();
bool sub_164_176();
bool sub_176_201();
bool sub_177_195();
bool sub_195_200();
bool sub_209_226();
bool sub_226_302();
bool sub_265_285();
bool sub_266_271();
bool sub_271_284();
bool sub_275_278();
bool sub_278_283();
bool sub_285_298();
bool sub_286_291();
bool sub_291_297();
bool sub_302_312();
bool sub_307_311();
bool sub_308_309();
bool sub_312_339();
bool sub_314_319();
bool sub_319_338();
bool sub_322_328();
bool sub_328_337();
bool sub_329_333();
bool sub_333_336();
bool sub_339_347();
bool sub_341_342();
bool sub_342_346();
bool sub_343_344();
bool sub_344_345();
bool sub_347_354();
bool sub_354_358();
bool sub_358_382();
bool sub_360_364();
bool sub_364_381();
bool sub_365_379();
bool sub_368_375();
bool sub_375_378();
bool sub_379_380();
bool sub_382_398();
bool sub_384_388();
bool sub_388_397();
bool sub_389_395();
bool sub_395_396();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    // 0: call
    {
        sub_34_209();
    }
    // 1: call
    {
        sub_302_312();
    }
    // 2: call
    {
        sub_312_339();
    }
    // 3: call
    {
        sub_358_382();
    }
    // 4: call
    {
        sub_382_398();
    }
    // 5: end
    return true;
}
bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 8: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 9: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 10: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 11: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 13: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 14: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 15: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 16: dp3
    reg_tmp4.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 17: dp3
    reg_tmp4.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 18: dp3
    reg_tmp4.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 19: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 22: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 23: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 24: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 25: dp3
    reg_tmp4.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 26: dp3
    reg_tmp4.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 27: dp3
    reg_tmp4.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 28: dp3
    reg_tmp5.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 29: dp3
    reg_tmp5.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 30: dp3
    reg_tmp5.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 31: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_safe(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_209() {
    // 34: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_142();
    } else {
        sub_142_208();
    }
    // 208: nop
    return false;
}
bool sub_40_142() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 42: mov
    reg_tmp7 = uniforms.f[93].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[93].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[93].xxxx;
    // 45: mul
    reg_tmp2 = mul_safe(uniforms.f[93].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 136: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 137: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 138: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 139: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 140: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 141: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 64: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 65: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 66: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 68: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 69: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 70: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 72: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 73: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 74: call
    {
        sub_209_226();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 60: callc
    if (conditional_code.y) {
        sub_12_21();
    }
    // 61: nop
    return false;
}
bool sub_76_135() {
    // 76: ifc
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    // 134: nop
    return false;
}
bool sub_77_109() {
    // 77: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 94: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 95: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 96: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 98: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 99: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 100: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 101: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 102: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 103: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 104: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 105: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 106: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 107: call
    {
        sub_226_302();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 126: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 127: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 128: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 130: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 131: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 132: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_142_208() {
    // 142: mov
    reg_tmp0 = uniforms.f[7];
    // 143: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 144: ifu
    if (uniforms.b[2]) {
        sub_145_155();
    } else {
        sub_155_163();
    }
    // 163: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_164_176();
    } else {
        sub_176_201();
    }
    // 201: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 202: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 203: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 204: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 205: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 206: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 207: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_145_155() {
    // 145: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 146: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 147: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 148: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 149: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 150: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 151: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 152: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 153: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 154: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_155_163() {
    // 155: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22], reg_tmp15), vec4(1.0));
    // 156: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23], reg_tmp15), vec4(1.0));
    // 157: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24], reg_tmp15), vec4(1.0));
    // 158: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 159: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 160: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 161: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 162: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_164_176() {
    // 164: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 165: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 166: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 167: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 168: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 169: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 170: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 171: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 172: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 173: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 174: call
    {
        sub_209_226();
    }
    // 175: nop
    return false;
}
bool sub_176_201() {
    // 176: ifc
    if (all(conditional_code)) {
        sub_177_195();
    } else {
        sub_195_200();
    }
    // 200: nop
    return false;
}
bool sub_177_195() {
    // 177: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 178: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 179: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 180: dp3
    reg_tmp11.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 181: dp3
    reg_tmp11.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 182: dp3
    reg_tmp11.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 183: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 184: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 185: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 186: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 187: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 188: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 189: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 190: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 191: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 192: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 193: call
    {
        sub_226_302();
    }
    // 194: nop
    return false;
}
bool sub_195_200() {
    // 195: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 196: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 197: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 198: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 199: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_209_226() {
    uint jmp_to = 209u;
    while (true) {
        switch (jmp_to) {
        case 209u: {
            // 209: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 210: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 211: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 212: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 213: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 214: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 215: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 216: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 225u; break; }
            }
            // 217: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 218: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 219: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 220: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 221: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 222: jmpc
            if (conditional_code.x) {
                { jmp_to = 225u; break; }
            }
            // 223: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 224: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 225u: {
            // 225: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_226_302() {
    uint jmp_to = 226u;
    while (true) {
        switch (jmp_to) {
        case 226u: {
            // 226: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 227: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 228: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 229: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 230: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 231: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 232: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 233: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 234: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 235: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 301u; break; }
            }
            // 236: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 237: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 238: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 239: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 240: dp3
            reg_tmp5.w = dot(vec3(mul_safe(reg_tmp5, reg_tmp5)), vec3(1.0));
            // 241: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 242: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 243: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 244: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 245: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 246: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 247: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 248: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 249: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 250: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 251: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 252: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 253: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 254: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 255: jmpc
            if (!conditional_code.x) {
                { jmp_to = 263u; break; }
            }
            // 256: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 257: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 258: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 259: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp7, reg_tmp7), vec4(1.0)));
            // 260: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 261: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 262: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 301u; break; }
            }
        }
        case 263u: {
            // 263: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 264: ifc
            if (conditional_code.x) {
                sub_265_285();
            } else {
                sub_285_298();
            }
            // 298: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp8, reg_tmp8), vec4(1.0)));
            // 299: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 300: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 301u: {
            // 301: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_265_285() {
    // 265: ifc
    if (conditional_code.y) {
        sub_266_271();
    } else {
        sub_271_284();
    }
    // 284: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_266_271() {
    // 266: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 267: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 268: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 269: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 270: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_271_284() {
    // 271: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 272: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 273: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 274: ifc
    if (conditional_code.x) {
        sub_275_278();
    } else {
        sub_278_283();
    }
    // 283: nop
    return false;
}
bool sub_275_278() {
    // 275: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 276: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 277: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_278_283() {
    // 278: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 279: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 280: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 281: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 282: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_285_298() {
    // 285: ifc
    if (conditional_code.y) {
        sub_286_291();
    } else {
        sub_291_297();
    }
    // 297: nop
    return false;
}
bool sub_286_291() {
    // 286: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 287: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 288: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 289: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 290: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_291_297() {
    // 291: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 292: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 293: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 294: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 295: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 296: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_302_312() {
    // 302: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 303: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 304: mov
    reg_tmp9 = uniforms.f[21];
    // 305: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 306: ifc
    if (conditional_code.y) {
        sub_307_311();
    }
    // 311: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_307_311() {
    // 307: ifu
    if (uniforms.b[7]) {
        sub_308_309();
    }
    // 309: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 310: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_308_309() {
    // 308: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_312_339() {
    // 312: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 313: ifu
    if (uniforms.b[9]) {
        sub_314_319();
    } else {
        sub_319_338();
    }
    // 338: nop
    return false;
}
bool sub_314_319() {
    // 314: call
    {
        sub_339_347();
    }
    // 315: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11].xywz, reg_tmp6), vec4(1.0));
    // 316: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12].xywz, reg_tmp6), vec4(1.0));
    // 317: mov
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    // 318: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_319_338() {
    // 319: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 320: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 321: ifc
    if (all(not(conditional_code))) {
        sub_322_328();
    } else {
        sub_328_337();
    }
    // 337: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_322_328() {
    // 322: mov
    reg_tmp6 = reg_tmp10;
    // 323: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11], reg_tmp6), vec4(1.0));
    // 324: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12], reg_tmp6), vec4(1.0));
    // 325: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[13], reg_tmp6), vec4(1.0));
    // 326: mul
    reg_tmp0.xy = (mul_safe(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 327: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_328_337() {
    // 328: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_329_333();
    } else {
        sub_333_336();
    }
    // 336: nop
    return false;
}
bool sub_329_333() {
    // 329: call
    {
        sub_347_354();
    }
    // 330: dp3
    reg_tmp3.x = dot(vec3(mul_safe(uniforms.f[11], reg_tmp6)), vec3(1.0));
    // 331: dp3
    reg_tmp3.y = dot(vec3(mul_safe(uniforms.f[12], reg_tmp6)), vec3(1.0));
    // 332: dp3
    reg_tmp3.z = dot(vec3(mul_safe(uniforms.f[13], reg_tmp6)), vec3(1.0));
    return false;
}
bool sub_333_336() {
    // 333: call
    {
        sub_354_358();
    }
    // 334: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11], reg_tmp6), vec4(1.0));
    // 335: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12], reg_tmp6), vec4(1.0));
    return false;
}
bool sub_339_347() {
    // 339: cmp
    conditional_code = equal(vec2(uniforms.f[93].yzzz), vec2(reg_tmp0.xyyy));
    // 340: ifc
    if (all(not(conditional_code))) {
        sub_341_342();
    } else {
        sub_342_346();
    }
    // 346: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_341_342() {
    // 341: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_342_346() {
    // 342: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_343_344();
    } else {
        sub_344_345();
    }
    // 345: nop
    return false;
}
bool sub_343_344() {
    // 343: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_344_345() {
    // 344: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_347_354() {
    // 347: mov
    reg_tmp2 = -reg_tmp15;
    // 348: dp3
    reg_tmp2.w = dot(vec3(mul_safe(reg_tmp2, reg_tmp2)), vec3(1.0));
    // 349: rsq
    reg_tmp2.w = rsq_safe(reg_tmp2.wwww.x);
    // 350: mul
    reg_tmp2 = mul_safe(reg_tmp2, reg_tmp2.wwww);
    // 351: dp3
    reg_tmp1 = vec4(dot(vec3(mul_safe(reg_tmp2, reg_tmp14)), vec3(1.0)));
    // 352: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 353: mad
    reg_tmp6 = fma_safe(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_354_358() {
    // 354: mov
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    // 355: mov
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    // 356: mad
    reg_tmp6 = fma_safe(reg_tmp14, reg_tmp1, reg_tmp1);
    // 357: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_358_382() {
    // 358: mov
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    // 359: ifu
    if (uniforms.b[10]) {
        sub_360_364();
    } else {
        sub_364_381();
    }
    // 381: nop
    return false;
}
bool sub_360_364() {
    // 360: call
    {
        sub_339_347();
    }
    // 361: dp4
    reg_tmp4.x = dot(mul_safe(uniforms.f[14].xywz, reg_tmp6), vec4(1.0));
    // 362: dp4
    reg_tmp4.y = dot(mul_safe(uniforms.f[15].xywz, reg_tmp6), vec4(1.0));
    // 363: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_364_381() {
    // 364: ifu
    if (uniforms.b[13]) {
        sub_365_379();
    } else {
        sub_379_380();
    }
    // 380: nop
    return false;
}
bool sub_365_379() {
    // 365: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 366: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 367: ifc
    if (all(not(conditional_code))) {
        sub_368_375();
    } else {
        sub_375_378();
    }
    // 378: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_368_375() {
    // 368: mov
    reg_tmp6 = reg_tmp10;
    // 369: dp4
    reg_tmp4.x = dot(mul_safe(uniforms.f[14], reg_tmp6), vec4(1.0));
    // 370: dp4
    reg_tmp4.y = dot(mul_safe(uniforms.f[15], reg_tmp6), vec4(1.0));
    // 371: dp4
    reg_tmp4.z = dot(mul_safe(uniforms.f[16], reg_tmp6), vec4(1.0));
    // 372: rcp
    reg_tmp6.w = rcp_safe(reg_tmp4.zzzz.x);
    // 373: mul
    reg_tmp4.xy = (mul_safe(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    // 374: add
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_375_378() {
    // 375: call
    {
        sub_354_358();
    }
    // 376: dp4
    reg_tmp4.x = dot(mul_safe(uniforms.f[14], reg_tmp6), vec4(1.0));
    // 377: dp4
    reg_tmp4.y = dot(mul_safe(uniforms.f[15], reg_tmp6), vec4(1.0));
    return false;
}
bool sub_379_380() {
    // 379: mov
    vs_out_attr5 = uniforms.f[93].xxxx;
    return false;
}
bool sub_382_398() {
    // 382: mov
    reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
    // 383: ifu
    if (uniforms.b[11]) {
        sub_384_388();
    } else {
        sub_388_397();
    }
    // 397: nop
    return false;
}
bool sub_384_388() {
    // 384: call
    {
        sub_339_347();
    }
    // 385: dp4
    reg_tmp5.x = dot(mul_safe(uniforms.f[17].xywz, reg_tmp6), vec4(1.0));
    // 386: dp4
    reg_tmp5.y = dot(mul_safe(uniforms.f[18].xywz, reg_tmp6), vec4(1.0));
    // 387: mov
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_388_397() {
    // 388: ifu
    if (uniforms.b[14]) {
        sub_389_395();
    } else {
        sub_395_396();
    }
    // 396: nop
    return false;
}
bool sub_389_395() {
    // 389: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 390: mov
    reg_tmp5.zw = (reg_tmp6.zwww).zw;
    // 391: call
    {
        sub_354_358();
    }
    // 392: dp4
    reg_tmp5.x = dot(mul_safe(uniforms.f[17], reg_tmp6), vec4(1.0));
    // 393: dp4
    reg_tmp5.y = dot(mul_safe(uniforms.f[18], reg_tmp6), vec4(1.0));
    // 394: mov
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_395_396() {
    // 395: mov
    vs_out_attr6 = uniforms.f[93].xxxx;
    return false;
}
// reference: 70B134B9A5E335CC, 4678A4FD452DD0E4
// shader: 8DD9, AD7922A63ED54CA7

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

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];
 in vec4 vs_out_attr4[];
 in vec4 vs_out_attr5[];
 in vec4 vs_out_attr6[];

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
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

    texcoord0_w = vtx.attributes[4].z;
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
// reference: FC74FA4ACA1C8C74, AD7922A63ED54CA7
// shader: 8B30, FF0287BA9B67882F

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

float ProcTexLookupLUT(int offset, float coord) {
    coord *= 128.0;
    float index_i = clamp(floor(coord), 0.0, 127.0);
    float index_f = coord - index_i; // fract() cannot be used here because 128.0 needs to be
                                     // extracted as index_i = 127.0 and index_f = 1.0
    vec2 entry = texelFetch(texture_buffer_lut_rg, int(index_i) + offset).rg;
    return clamp(entry.r + entry.g * index_f, 0.0, 1.0);
}
    
int ProcTexNoiseRand1D(int v) {
    const int table[] = int[](0,4,10,8,4,9,7,12,5,15,13,14,11,15,2,11);
    return ((v % 9 + 2) * 3 & 0xF) ^ table[(v / 9) & 0xF];
}

float ProcTexNoiseRand2D(vec2 point) {
    const int table[] = int[](10,2,15,8,0,7,4,5,5,13,2,6,13,9,3,14);
    int u2 = ProcTexNoiseRand1D(int(point.x));
    int v2 = ProcTexNoiseRand1D(int(point.y));
    v2 += ((u2 & 3) == 1) ? 4 : 0;
    v2 ^= (u2 & 1) * 6;
    v2 += 10 + u2;
    v2 &= 0xF;
    v2 ^= table[u2];
    return -1.0 + float(v2) * (2.0 / 15.0);
}

float ProcTexNoiseCoef(vec2 x) {
    vec2 grid  = 9.0 * proctex_noise_f * abs(x + proctex_noise_p);
    vec2 point = floor(grid);
    vec2 frac  = grid - point;

    float g0 = ProcTexNoiseRand2D(point) * (frac.x + frac.y);
    float g1 = ProcTexNoiseRand2D(point + vec2(1.0, 0.0)) * (frac.x + frac.y - 1.0);
    float g2 = ProcTexNoiseRand2D(point + vec2(0.0, 1.0)) * (frac.x + frac.y - 1.0);
    float g3 = ProcTexNoiseRand2D(point + vec2(1.0, 1.0)) * (frac.x + frac.y - 2.0);

    float x_noise = ProcTexLookupLUT(proctex_noise_lut_offset, frac.x);
    float y_noise = ProcTexLookupLUT(proctex_noise_lut_offset, frac.y);
    float x0 = mix(g0, g1, x_noise);
    float x1 = mix(g2, g3, x_noise);
    return mix(x0, x1, y_noise);
}
        vec4 SampleProcTexColor(float lut_coord, int level) {
int lut_width = 128 >> level;
int lut_offsets[8] = int[](0, 128, 192, 224, 0xF0, 0xF8, 0xFC, 0xFE);
int lut_offset = lut_offsets[level];
lut_coord *= float(lut_width - 1);
lut_coord += float(lut_offset);
return texelFetch(texture_buffer_lut_rgba, int(round(lut_coord)) + proctex_lut_offset);
}
vec4 ProcTex() {
vec2 uv = abs(texcoord0);
float u_shift = 0.0;
float v_shift = 0.0;
uv += proctex_noise_a * ProcTexNoiseCoef(uv);
uv = abs(uv);
float u = uv.x + u_shift;
float v = uv.y + v_shift;
u = mix(1.0 - fract(u), fract(u), int(u) % 2 == 0);
v = mix(1.0 - fract(v), fract(v), int(v) % 2 == 0);
float lut_coord = ProcTexLookupLUT(proctex_color_map_offset, ((u + v) * 0.5));
vec4 final_color = SampleProcTexColor(lut_coord, 0);
return final_color;
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor1).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.ggg) * (const_color[0].rgb) + (ProcTex().rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor2.r) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb) - vec3(0.5), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((vec3(1.0) - ProcTex().rrr) * (secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (vec3(1.0) - rounded_primary_color.bbb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 10CCADFC8C241709, FF0287BA9B67882F
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, FF0287BA9B67882F
// shader: 8B30, B038D2DABE700107

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0A45E4E6AAE3F59A, B038D2DABE700107
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, B038D2DABE700107
// shader: 8B31, F90A89AAB0E37EE2

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
layout(location = 7) in vec4 vs_in_reg7;
layout(location = 8) in vec4 vs_in_reg8;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_7_12();
bool sub_12_21();
bool sub_21_34();
bool sub_34_209();
bool sub_40_142();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_142_208();
bool sub_145_155();
bool sub_155_163();
bool sub_164_176();
bool sub_176_201();
bool sub_177_195();
bool sub_195_200();
bool sub_209_226();
bool sub_226_302();
bool sub_265_285();
bool sub_266_271();
bool sub_271_284();
bool sub_275_278();
bool sub_278_283();
bool sub_285_298();
bool sub_286_291();
bool sub_291_297();
bool sub_302_312();
bool sub_307_311();
bool sub_308_309();
bool sub_409_4096();

bool exec_shader() {
    sub_409_4096();
    return true;
}

bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 8: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 9: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 10: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 11: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 13: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 14: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 15: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 16: dp3
    reg_tmp4.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 17: dp3
    reg_tmp4.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 18: dp3
    reg_tmp4.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 19: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 22: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 23: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 24: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 25: dp3
    reg_tmp4.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 26: dp3
    reg_tmp4.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 27: dp3
    reg_tmp4.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 28: dp3
    reg_tmp5.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 29: dp3
    reg_tmp5.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 30: dp3
    reg_tmp5.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 31: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_safe(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_209() {
    // 34: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_142();
    } else {
        sub_142_208();
    }
    // 208: nop
    return false;
}
bool sub_40_142() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 42: mov
    reg_tmp7 = uniforms.f[93].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[93].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[93].xxxx;
    // 45: mul
    reg_tmp2 = mul_safe(uniforms.f[93].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 136: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 137: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 138: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 139: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 140: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 141: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 64: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 65: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 66: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 68: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 69: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 70: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 72: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 73: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 74: call
    {
        sub_209_226();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 60: callc
    if (conditional_code.y) {
        sub_12_21();
    }
    // 61: nop
    return false;
}
bool sub_76_135() {
    // 76: ifc
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    // 134: nop
    return false;
}
bool sub_77_109() {
    // 77: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 94: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 95: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 96: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 98: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 99: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 100: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 101: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 102: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 103: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 104: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 105: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 106: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 107: call
    {
        sub_226_302();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 126: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 127: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 128: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 130: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 131: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 132: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_142_208() {
    // 142: mov
    reg_tmp0 = uniforms.f[7];
    // 143: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 144: ifu
    if (uniforms.b[2]) {
        sub_145_155();
    } else {
        sub_155_163();
    }
    // 163: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_164_176();
    } else {
        sub_176_201();
    }
    // 201: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 202: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 203: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 204: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 205: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 206: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 207: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_145_155() {
    // 145: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 146: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 147: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 148: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 149: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 150: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 151: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 152: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 153: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 154: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_155_163() {
    // 155: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22], reg_tmp15), vec4(1.0));
    // 156: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23], reg_tmp15), vec4(1.0));
    // 157: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24], reg_tmp15), vec4(1.0));
    // 158: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 159: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 160: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 161: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 162: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_164_176() {
    // 164: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 165: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 166: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 167: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 168: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 169: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 170: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 171: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 172: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 173: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 174: call
    {
        sub_209_226();
    }
    // 175: nop
    return false;
}
bool sub_176_201() {
    // 176: ifc
    if (all(conditional_code)) {
        sub_177_195();
    } else {
        sub_195_200();
    }
    // 200: nop
    return false;
}
bool sub_177_195() {
    // 177: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 178: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 179: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 180: dp3
    reg_tmp11.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 181: dp3
    reg_tmp11.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 182: dp3
    reg_tmp11.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 183: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 184: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 185: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 186: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 187: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 188: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 189: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 190: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 191: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 192: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 193: call
    {
        sub_226_302();
    }
    // 194: nop
    return false;
}
bool sub_195_200() {
    // 195: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 196: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 197: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 198: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 199: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_209_226() {
    uint jmp_to = 209u;
    while (true) {
        switch (jmp_to) {
        case 209u: {
            // 209: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 210: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 211: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 212: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 213: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 214: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 215: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 216: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 225u; break; }
            }
            // 217: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 218: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 219: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 220: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 221: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 222: jmpc
            if (conditional_code.x) {
                { jmp_to = 225u; break; }
            }
            // 223: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 224: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 225u: {
            // 225: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_226_302() {
    uint jmp_to = 226u;
    while (true) {
        switch (jmp_to) {
        case 226u: {
            // 226: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 227: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 228: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 229: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 230: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 231: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 232: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 233: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 234: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 235: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 301u; break; }
            }
            // 236: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 237: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 238: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 239: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 240: dp3
            reg_tmp5.w = dot(vec3(mul_safe(reg_tmp5, reg_tmp5)), vec3(1.0));
            // 241: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 242: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 243: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 244: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 245: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 246: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 247: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 248: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 249: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 250: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 251: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 252: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 253: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 254: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 255: jmpc
            if (!conditional_code.x) {
                { jmp_to = 263u; break; }
            }
            // 256: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 257: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 258: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 259: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp7, reg_tmp7), vec4(1.0)));
            // 260: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 261: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 262: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 301u; break; }
            }
        }
        case 263u: {
            // 263: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 264: ifc
            if (conditional_code.x) {
                sub_265_285();
            } else {
                sub_285_298();
            }
            // 298: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp8, reg_tmp8), vec4(1.0)));
            // 299: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 300: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 301u: {
            // 301: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_265_285() {
    // 265: ifc
    if (conditional_code.y) {
        sub_266_271();
    } else {
        sub_271_284();
    }
    // 284: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_266_271() {
    // 266: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 267: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 268: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 269: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 270: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_271_284() {
    // 271: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 272: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 273: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 274: ifc
    if (conditional_code.x) {
        sub_275_278();
    } else {
        sub_278_283();
    }
    // 283: nop
    return false;
}
bool sub_275_278() {
    // 275: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 276: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 277: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_278_283() {
    // 278: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 279: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 280: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 281: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 282: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_285_298() {
    // 285: ifc
    if (conditional_code.y) {
        sub_286_291();
    } else {
        sub_291_297();
    }
    // 297: nop
    return false;
}
bool sub_286_291() {
    // 286: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 287: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 288: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 289: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 290: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_291_297() {
    // 291: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 292: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 293: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 294: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 295: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 296: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_302_312() {
    // 302: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 303: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 304: mov
    reg_tmp9 = uniforms.f[21];
    // 305: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 306: ifc
    if (conditional_code.y) {
        sub_307_311();
    }
    // 311: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_307_311() {
    // 307: ifu
    if (uniforms.b[7]) {
        sub_308_309();
    }
    // 309: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 310: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_308_309() {
    // 308: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_409_4096() {
    // 409: call
    {
        sub_34_209();
    }
    // 410: call
    {
        sub_302_312();
    }
    // 411: end
    return true;
}
// reference: 4785A195F3EB9A62, F90A89AAB0E37EE2
// shader: 8DD9, 4BD70AD09292A3DA

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

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];

struct Vertex {
    vec4 attributes[4];
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
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(0.0, 0.0);
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
    prim_buffer[0].attributes = vec4[4](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0]);
    prim_buffer[1].attributes = vec4[4](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1]);
    prim_buffer[2].attributes = vec4[4](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 3901EC4BEC56958E, 4BD70AD09292A3DA
// shader: 8B30, 4982B44D494E20C9

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D4F4BEF9C3444BB3, 4982B44D494E20C9
// program: F90A89AAB0E37EE2, 4BD70AD09292A3DA, 4982B44D494E20C9
// shader: 8B30, 273644D90ADF2711

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 62769A258D327013, 273644D90ADF2711
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 273644D90ADF2711
// shader: 8B30, 9F4472EE66D05307

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((rounded_primary_color.aaa) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3AFDFBAD14DBCCBC, 9F4472EE66D05307
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 9F4472EE66D05307
// shader: 8B30, 124DD61C45CF942F

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((1.0 - texcolor2.g) + (texcolor0.g), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp(min((last_tex_env_out.aaa) + (const_color[1].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1AF23575D6C30209, 124DD61C45CF942F
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, 124DD61C45CF942F
// reference: 70B134B97E5A948B, 4678A4FD452DD0E4
// reference: F54916762DE46B42, 8809BF824E86BD32
// shader: 8B30, C59EA4758EB9FB8A

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 10807477801C7747, C59EA4758EB9FB8A
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, C59EA4758EB9FB8A
// shader: 8B30, BE4C56FF608580E6

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 10807477C5022927, BE4C56FF608580E6
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, BE4C56FF608580E6
// reference: 108074777BFC06FE, BE4C56FF608580E6
// shader: 8B30, E7E676F465F0504F

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ABB514A34FE115FA, E7E676F465F0504F
// program: F90A89AAB0E37EE2, 4BD70AD09292A3DA, E7E676F465F0504F
// shader: 8B31, 0817F9534CBA084E

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
layout(location = 8) in vec4 vs_in_reg8;

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

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_7_12();
bool sub_12_21();
bool sub_21_34();
bool sub_34_209();
bool sub_40_142();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_142_208();
bool sub_145_155();
bool sub_155_163();
bool sub_164_176();
bool sub_176_201();
bool sub_177_195();
bool sub_195_200();
bool sub_209_226();
bool sub_226_302();
bool sub_265_285();
bool sub_266_271();
bool sub_271_284();
bool sub_275_278();
bool sub_278_283();
bool sub_285_298();
bool sub_286_291();
bool sub_291_297();
bool sub_302_312();
bool sub_307_311();
bool sub_308_309();
bool sub_312_339();
bool sub_314_319();
bool sub_319_338();
bool sub_322_328();
bool sub_328_337();
bool sub_329_333();
bool sub_333_336();
bool sub_339_347();
bool sub_341_342();
bool sub_342_346();
bool sub_343_344();
bool sub_344_345();
bool sub_347_354();
bool sub_354_358();
bool sub_358_382();
bool sub_360_364();
bool sub_364_381();
bool sub_365_379();
bool sub_368_375();
bool sub_375_378();
bool sub_379_380();
bool sub_398_4096();

bool exec_shader() {
    sub_398_4096();
    return true;
}

bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 8: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 9: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 10: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 11: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 13: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 14: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 15: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 16: dp3
    reg_tmp4.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 17: dp3
    reg_tmp4.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 18: dp3
    reg_tmp4.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 19: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 22: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 23: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 24: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 25: dp3
    reg_tmp4.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 26: dp3
    reg_tmp4.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 27: dp3
    reg_tmp4.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 28: dp3
    reg_tmp5.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 29: dp3
    reg_tmp5.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 30: dp3
    reg_tmp5.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 31: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_safe(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_209() {
    // 34: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_142();
    } else {
        sub_142_208();
    }
    // 208: nop
    return false;
}
bool sub_40_142() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 42: mov
    reg_tmp7 = uniforms.f[93].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[93].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[93].xxxx;
    // 45: mul
    reg_tmp2 = mul_safe(uniforms.f[93].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 136: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 137: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 138: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 139: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 140: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 141: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 64: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 65: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 66: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 68: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 69: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 70: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 72: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 73: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 74: call
    {
        sub_209_226();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 60: callc
    if (conditional_code.y) {
        sub_12_21();
    }
    // 61: nop
    return false;
}
bool sub_76_135() {
    // 76: ifc
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    // 134: nop
    return false;
}
bool sub_77_109() {
    // 77: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 94: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 95: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 96: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 98: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 99: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 100: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 101: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 102: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 103: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 104: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 105: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 106: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 107: call
    {
        sub_226_302();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 126: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 127: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 128: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 130: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 131: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 132: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_142_208() {
    // 142: mov
    reg_tmp0 = uniforms.f[7];
    // 143: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 144: ifu
    if (uniforms.b[2]) {
        sub_145_155();
    } else {
        sub_155_163();
    }
    // 163: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_164_176();
    } else {
        sub_176_201();
    }
    // 201: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 202: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 203: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 204: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 205: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 206: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 207: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_145_155() {
    // 145: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 146: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 147: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 148: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 149: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 150: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 151: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 152: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 153: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 154: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_155_163() {
    // 155: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22], reg_tmp15), vec4(1.0));
    // 156: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23], reg_tmp15), vec4(1.0));
    // 157: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24], reg_tmp15), vec4(1.0));
    // 158: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 159: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 160: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 161: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 162: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_164_176() {
    // 164: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 165: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 166: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 167: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 168: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 169: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 170: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 171: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 172: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 173: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 174: call
    {
        sub_209_226();
    }
    // 175: nop
    return false;
}
bool sub_176_201() {
    // 176: ifc
    if (all(conditional_code)) {
        sub_177_195();
    } else {
        sub_195_200();
    }
    // 200: nop
    return false;
}
bool sub_177_195() {
    // 177: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 178: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 179: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 180: dp3
    reg_tmp11.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 181: dp3
    reg_tmp11.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 182: dp3
    reg_tmp11.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 183: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 184: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 185: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 186: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 187: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 188: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 189: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 190: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 191: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 192: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 193: call
    {
        sub_226_302();
    }
    // 194: nop
    return false;
}
bool sub_195_200() {
    // 195: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 196: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 197: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 198: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 199: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_209_226() {
    uint jmp_to = 209u;
    while (true) {
        switch (jmp_to) {
        case 209u: {
            // 209: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 210: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 211: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 212: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 213: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 214: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 215: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 216: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 225u; break; }
            }
            // 217: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 218: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 219: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 220: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 221: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 222: jmpc
            if (conditional_code.x) {
                { jmp_to = 225u; break; }
            }
            // 223: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 224: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 225u: {
            // 225: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_226_302() {
    uint jmp_to = 226u;
    while (true) {
        switch (jmp_to) {
        case 226u: {
            // 226: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 227: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 228: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 229: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 230: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 231: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 232: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 233: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 234: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 235: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 301u; break; }
            }
            // 236: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 237: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 238: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 239: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 240: dp3
            reg_tmp5.w = dot(vec3(mul_safe(reg_tmp5, reg_tmp5)), vec3(1.0));
            // 241: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 242: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 243: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 244: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 245: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 246: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 247: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 248: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 249: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 250: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 251: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 252: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 253: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 254: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 255: jmpc
            if (!conditional_code.x) {
                { jmp_to = 263u; break; }
            }
            // 256: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 257: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 258: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 259: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp7, reg_tmp7), vec4(1.0)));
            // 260: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 261: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 262: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 301u; break; }
            }
        }
        case 263u: {
            // 263: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 264: ifc
            if (conditional_code.x) {
                sub_265_285();
            } else {
                sub_285_298();
            }
            // 298: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp8, reg_tmp8), vec4(1.0)));
            // 299: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 300: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 301u: {
            // 301: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_265_285() {
    // 265: ifc
    if (conditional_code.y) {
        sub_266_271();
    } else {
        sub_271_284();
    }
    // 284: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_266_271() {
    // 266: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 267: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 268: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 269: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 270: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_271_284() {
    // 271: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 272: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 273: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 274: ifc
    if (conditional_code.x) {
        sub_275_278();
    } else {
        sub_278_283();
    }
    // 283: nop
    return false;
}
bool sub_275_278() {
    // 275: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 276: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 277: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_278_283() {
    // 278: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 279: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 280: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 281: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 282: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_285_298() {
    // 285: ifc
    if (conditional_code.y) {
        sub_286_291();
    } else {
        sub_291_297();
    }
    // 297: nop
    return false;
}
bool sub_286_291() {
    // 286: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 287: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 288: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 289: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 290: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_291_297() {
    // 291: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 292: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 293: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 294: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 295: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 296: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_302_312() {
    // 302: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 303: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 304: mov
    reg_tmp9 = uniforms.f[21];
    // 305: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 306: ifc
    if (conditional_code.y) {
        sub_307_311();
    }
    // 311: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_307_311() {
    // 307: ifu
    if (uniforms.b[7]) {
        sub_308_309();
    }
    // 309: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 310: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_308_309() {
    // 308: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_312_339() {
    // 312: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 313: ifu
    if (uniforms.b[9]) {
        sub_314_319();
    } else {
        sub_319_338();
    }
    // 338: nop
    return false;
}
bool sub_314_319() {
    // 314: call
    {
        sub_339_347();
    }
    // 315: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11].xywz, reg_tmp6), vec4(1.0));
    // 316: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12].xywz, reg_tmp6), vec4(1.0));
    // 317: mov
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    // 318: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_319_338() {
    // 319: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 320: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 321: ifc
    if (all(not(conditional_code))) {
        sub_322_328();
    } else {
        sub_328_337();
    }
    // 337: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_322_328() {
    // 322: mov
    reg_tmp6 = reg_tmp10;
    // 323: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11], reg_tmp6), vec4(1.0));
    // 324: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12], reg_tmp6), vec4(1.0));
    // 325: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[13], reg_tmp6), vec4(1.0));
    // 326: mul
    reg_tmp0.xy = (mul_safe(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 327: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_328_337() {
    // 328: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_329_333();
    } else {
        sub_333_336();
    }
    // 336: nop
    return false;
}
bool sub_329_333() {
    // 329: call
    {
        sub_347_354();
    }
    // 330: dp3
    reg_tmp3.x = dot(vec3(mul_safe(uniforms.f[11], reg_tmp6)), vec3(1.0));
    // 331: dp3
    reg_tmp3.y = dot(vec3(mul_safe(uniforms.f[12], reg_tmp6)), vec3(1.0));
    // 332: dp3
    reg_tmp3.z = dot(vec3(mul_safe(uniforms.f[13], reg_tmp6)), vec3(1.0));
    return false;
}
bool sub_333_336() {
    // 333: call
    {
        sub_354_358();
    }
    // 334: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11], reg_tmp6), vec4(1.0));
    // 335: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12], reg_tmp6), vec4(1.0));
    return false;
}
bool sub_339_347() {
    // 339: cmp
    conditional_code = equal(vec2(uniforms.f[93].yzzz), vec2(reg_tmp0.xyyy));
    // 340: ifc
    if (all(not(conditional_code))) {
        sub_341_342();
    } else {
        sub_342_346();
    }
    // 346: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_341_342() {
    // 341: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_342_346() {
    // 342: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_343_344();
    } else {
        sub_344_345();
    }
    // 345: nop
    return false;
}
bool sub_343_344() {
    // 343: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_344_345() {
    // 344: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_347_354() {
    // 347: mov
    reg_tmp2 = -reg_tmp15;
    // 348: dp3
    reg_tmp2.w = dot(vec3(mul_safe(reg_tmp2, reg_tmp2)), vec3(1.0));
    // 349: rsq
    reg_tmp2.w = rsq_safe(reg_tmp2.wwww.x);
    // 350: mul
    reg_tmp2 = mul_safe(reg_tmp2, reg_tmp2.wwww);
    // 351: dp3
    reg_tmp1 = vec4(dot(vec3(mul_safe(reg_tmp2, reg_tmp14)), vec3(1.0)));
    // 352: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 353: mad
    reg_tmp6 = fma_safe(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_354_358() {
    // 354: mov
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    // 355: mov
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    // 356: mad
    reg_tmp6 = fma_safe(reg_tmp14, reg_tmp1, reg_tmp1);
    // 357: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_358_382() {
    // 358: mov
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    // 359: ifu
    if (uniforms.b[10]) {
        sub_360_364();
    } else {
        sub_364_381();
    }
    // 381: nop
    return false;
}
bool sub_360_364() {
    // 360: call
    {
        sub_339_347();
    }
    // 361: dp4
    reg_tmp4.x = dot(mul_safe(uniforms.f[14].xywz, reg_tmp6), vec4(1.0));
    // 362: dp4
    reg_tmp4.y = dot(mul_safe(uniforms.f[15].xywz, reg_tmp6), vec4(1.0));
    // 363: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_364_381() {
    // 364: ifu
    if (uniforms.b[13]) {
        sub_365_379();
    } else {
        sub_379_380();
    }
    // 380: nop
    return false;
}
bool sub_365_379() {
    // 365: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 366: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 367: ifc
    if (all(not(conditional_code))) {
        sub_368_375();
    } else {
        sub_375_378();
    }
    // 378: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_368_375() {
    // 368: mov
    reg_tmp6 = reg_tmp10;
    // 369: dp4
    reg_tmp4.x = dot(mul_safe(uniforms.f[14], reg_tmp6), vec4(1.0));
    // 370: dp4
    reg_tmp4.y = dot(mul_safe(uniforms.f[15], reg_tmp6), vec4(1.0));
    // 371: dp4
    reg_tmp4.z = dot(mul_safe(uniforms.f[16], reg_tmp6), vec4(1.0));
    // 372: rcp
    reg_tmp6.w = rcp_safe(reg_tmp4.zzzz.x);
    // 373: mul
    reg_tmp4.xy = (mul_safe(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    // 374: add
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_375_378() {
    // 375: call
    {
        sub_354_358();
    }
    // 376: dp4
    reg_tmp4.x = dot(mul_safe(uniforms.f[14], reg_tmp6), vec4(1.0));
    // 377: dp4
    reg_tmp4.y = dot(mul_safe(uniforms.f[15], reg_tmp6), vec4(1.0));
    return false;
}
bool sub_379_380() {
    // 379: mov
    vs_out_attr5 = uniforms.f[93].xxxx;
    return false;
}
bool sub_398_4096() {
    // 398: call
    {
        sub_34_209();
    }
    // 399: call
    {
        sub_302_312();
    }
    // 400: call
    {
        sub_312_339();
    }
    // 401: call
    {
        sub_358_382();
    }
    // 402: end
    return true;
}
// reference: DFF5A6FE6A79EE8E, 0817F9534CBA084E
// shader: 8DD9, 3B3AE026C742C7D5

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
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

    texcoord0_w = vtx.attributes[4].z;
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
    prim_buffer[0].attributes = vec4[6](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0]);
    prim_buffer[1].attributes = vec4[6](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1]);
    prim_buffer[2].attributes = vec4[6](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 46A0C2E6B155D5CD, 3B3AE026C742C7D5
// shader: 8B30, 5C7B31B1941AC301

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (rounded_primary_color.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp(min((texcolor0.a) + (texcolor1.g), 1.0) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1.0) - rounded_primary_color.ggg) + (const_color[1].rgb) * (vec3(1.0) - (vec3(1.0) - rounded_primary_color.ggg)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1.0) - rounded_primary_color.bbb) + (const_color[2].rgb) * (vec3(1.0) - (vec3(1.0) - rounded_primary_color.bbb)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 07FFBB629FA644EA, 5C7B31B1941AC301
// program: 0817F9534CBA084E, 3B3AE026C742C7D5, 5C7B31B1941AC301
// shader: 8B30, EA5E28DA420D4997

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb) + (texcolor2.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp(min((texcolor1.rgb) + (const_color[1].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4A3353029ED57796, EA5E28DA420D4997
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, EA5E28DA420D4997
// shader: 8B30, 608860C78256AB9B

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 10807477778EDECB, 608860C78256AB9B
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 608860C78256AB9B
// shader: 8B30, 5121C8718256AB9B

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 77E52FD6778EDECB, 5121C8718256AB9B
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 5121C8718256AB9B
// shader: 8B30, 327A57EE33BE2826

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa) + (const_color[1].rgb) * (vec3(1.0) - (texcolor0.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 58D5A7B9A9EA0D66, 327A57EE33BE2826
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 327A57EE33BE2826
// shader: 8B30, 891DC8E84DC6467D

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 682266BFC9274AB1, 891DC8E84DC6467D
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 891DC8E84DC6467D
// shader: 8B30, FA0E09FA4DC6467D

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0F473D1EC9274AB1, FA0E09FA4DC6467D
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, FA0E09FA4DC6467D
// shader: 8B31, 0797E835195908AB

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

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;
 out vec4 vs_out_attr4;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr4 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_0_98();
bool sub_6_51();
bool sub_13_14();
bool sub_21_22();
bool sub_23_25();
bool sub_32_33();
bool sub_34_35();
bool sub_41_42();
bool sub_43_44();
bool sub_49_50();
bool sub_53_70();
bool sub_58_61();
bool sub_61_69();
bool sub_63_64();
bool sub_64_65();
bool sub_66_67();
bool sub_67_68();
bool sub_70_97();
bool sub_71_94();
bool sub_75_80();
bool sub_80_93();
bool sub_84_88();
bool sub_85_86();
bool sub_86_87();
bool sub_88_92();
bool sub_89_90();
bool sub_90_91();
bool sub_94_96();
bool sub_98_134();
bool sub_108_119();
bool sub_119_123();
bool sub_134_4096();
bool sub_145_149();
bool sub_149_153();
bool sub_155_157();
bool sub_157_179();
bool sub_167_171();
bool sub_188_190();
bool sub_191_192();
bool sub_201_203();
bool sub_204_205();
bool sub_214_216();
bool sub_217_218();
bool sub_235_244();
bool sub_244_253();

bool exec_shader() {
    sub_134_4096();
    return true;
}

bool sub_0_98() {
    // 0: mova
    address_registers.y = (ivec2(reg_tmp11.zzzz)).y;
    // 1: nop
    // 2: flr
    reg_tmp13 = floor(reg_tmp0.xxxx);
    // 3: add
    reg_tmp13 = reg_tmp0.xxxx + -reg_tmp13;
    // 4: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp13.xxxx));
    // 5: ifc
    if (conditional_code.x) {
        sub_6_51();
    }
    // 51: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].yyyy), vec2(reg_tmp11.xyyy));
    // 52: ifc
    if (!conditional_code.y) {
        sub_53_70();
    } else {
        sub_70_97();
    }
    // 97: nop
    return false;
}
bool sub_6_51() {
    // 6: add
    reg_tmp12.xy = (uniforms.f[5].xyyy + vs_in_reg0.zwww).xy;
    // 7: mov
    reg_tmp14.xy = (uniforms.f[6].wzzz).xy;
    // 8: mul
    reg_tmp13.xy = (mul_safe(uniforms.f[5].zzzz, reg_tmp0.xxxx)).xy;
    // 9: flr
    reg_tmp13.y = (floor(reg_tmp13)).y;
    // 10: add
    reg_tmp13.x = (reg_tmp13.xxxx + -reg_tmp13.yyyy).x;
    // 11: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp13.xxxx));
    // 12: ifc
    if (conditional_code.x) {
        sub_13_14();
    }
    // 14: mul
    reg_tmp14.xy = (mul_safe(reg_tmp14, reg_tmp2)).xy;
    // 15: mul
    reg_tmp13.x = (mul_safe(uniforms.f[5].zzzz, reg_tmp13.xxxx)).x;
    // 16: mul
    reg_tmp13 = mul_safe(uniforms.f[5].zyzy, reg_tmp13.xxxx);
    // 17: flr
    reg_tmp13.zw = (floor(reg_tmp13)).zw;
    // 18: add
    reg_tmp13.xy = (reg_tmp13.xyyy + -reg_tmp13.zwww).xy;
    // 19: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp13.xyyy));
    // 20: ifc
    if (conditional_code.y) {
        sub_21_22();
    }
    // 22: ifc
    if (conditional_code.x) {
        sub_23_25();
    }
    // 25: add
    reg_tmp14.xy = (uniforms.f[5].yyyy + -reg_tmp14.xyyy).xy;
    // 26: mul
    reg_tmp13.x = (mul_safe(uniforms.f[5].zzzz, reg_tmp13.xxxx)).x;
    // 27: mul
    reg_tmp13 = mul_safe(uniforms.f[5].zyzy, reg_tmp13.xxxx);
    // 28: flr
    reg_tmp13.zw = (floor(reg_tmp13)).zw;
    // 29: add
    reg_tmp13.xy = (reg_tmp13.xyyy + -reg_tmp13.zwww).xy;
    // 30: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp13.xyyy));
    // 31: ifc
    if (conditional_code.y) {
        sub_32_33();
    }
    // 33: ifc
    if (conditional_code.x) {
        sub_34_35();
    }
    // 35: mul
    reg_tmp13.x = (mul_safe(uniforms.f[5].zzzz, reg_tmp13.xxxx)).x;
    // 36: mul
    reg_tmp13 = mul_safe(uniforms.f[5].zyzy, reg_tmp13.xxxx);
    // 37: flr
    reg_tmp13.zw = (floor(reg_tmp13)).zw;
    // 38: add
    reg_tmp13.xy = (reg_tmp13.xyyy + -reg_tmp13.zwww).xy;
    // 39: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp13.xyyy));
    // 40: ifc
    if (conditional_code.y) {
        sub_41_42();
    }
    // 42: ifc
    if (conditional_code.x) {
        sub_43_44();
    }
    // 44: mul
    reg_tmp13.xy = (mul_safe(uniforms.f[5].zzzz, reg_tmp0.xxxx)).xy;
    // 45: flr
    reg_tmp13.y = (floor(reg_tmp13)).y;
    // 46: add
    reg_tmp13.x = (reg_tmp13.xxxx + -reg_tmp13.yyyy).x;
    // 47: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp13.xxxx));
    // 48: ifc
    if (conditional_code.x) {
        sub_49_50();
    }
    // 50: add
    reg_tmp12.y = (uniforms.f[5].yyyy + -reg_tmp12.yyyy).y;
    return false;
}
bool sub_13_14() {
    // 13: mov
    reg_tmp14.xy = (reg_tmp14.yxxx).xy;
    return false;
}
bool sub_21_22() {
    // 21: mul
    reg_tmp12.x = (mul_safe(reg_tmp12.xxxx, reg_tmp14.xxxx)).x;
    return false;
}
bool sub_23_25() {
    // 23: madi
    reg_tmp12.y = (fma_safe(reg_tmp12.yyyy, reg_tmp14.yyyy, uniforms.f[5].yyyy)).y;
    // 24: add
    reg_tmp12.y = (reg_tmp12.yyyy + -reg_tmp14.yyyy).y;
    return false;
}
bool sub_32_33() {
    // 32: add
    reg_tmp12.x = (reg_tmp12.xxxx + reg_tmp14.xxxx).x;
    return false;
}
bool sub_34_35() {
    // 34: add
    reg_tmp12.y = (reg_tmp12.yyyy + -reg_tmp14.yyyy).y;
    return false;
}
bool sub_41_42() {
    // 41: add
    reg_tmp12.x = (uniforms.f[5].yyyy + -reg_tmp12.xxxx).x;
    return false;
}
bool sub_43_44() {
    // 43: add
    reg_tmp12.y = (uniforms.f[5].yyyy + -reg_tmp12.yyyy).y;
    return false;
}
bool sub_49_50() {
    // 49: add
    reg_tmp12.xy = (uniforms.f[5].yyyy + -reg_tmp12.yxxx).xy;
    return false;
}
bool sub_53_70() {
    // 53: flr
    reg_tmp13.x = (floor(reg_tmp0.xxxx)).x;
    // 54: add
    reg_tmp13.x = (reg_tmp0.xxxx + -reg_tmp13).x;
    // 55: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp13.xxxx));
    // 56: mov
    reg_tmp13 = uniforms.f[32 + address_registers.y].wzyx;
    // 57: ifc
    if (conditional_code.x) {
        sub_58_61();
    } else {
        sub_61_69();
    }
    // 69: add
    reg_tmp11.z = (uniforms.f[5].yyyy + reg_tmp11.zzzz).z;
    return false;
}
bool sub_58_61() {
    // 58: mad
    reg_tmp11.xy = (fma_safe(reg_tmp12.xyyy, reg_tmp13.xyyy, reg_tmp13.zwww)).xy;
    // 59: mul
    reg_tmp11.xy = (mul_safe(reg_tmp11.xyyy, reg_tmp14.zwww)).xy;
    // 60: add
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_61_69() {
    // 61: cmp
    conditional_code = notEqual(vec2(uniforms.f[5].xxxx), vec2(vs_in_reg0.zwww));
    // 62: ifc
    if (!conditional_code.x) {
        sub_63_64();
    } else {
        sub_64_65();
    }
    // 65: ifc
    if (!conditional_code.y) {
        sub_66_67();
    } else {
        sub_67_68();
    }
    // 68: nop
    return false;
}
bool sub_63_64() {
    // 63: mov
    reg_tmp11.x = (reg_tmp13.xxxx).x;
    return false;
}
bool sub_64_65() {
    // 64: mov
    reg_tmp11.x = (reg_tmp13.zzzz).x;
    return false;
}
bool sub_66_67() {
    // 66: mov
    reg_tmp11.y = (reg_tmp13.yyyy).y;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp11.y = (reg_tmp13.wwww).y;
    return false;
}
bool sub_70_97() {
    // 70: ifc
    if (!conditional_code.x) {
        sub_71_94();
    } else {
        sub_94_96();
    }
    // 96: add
    reg_tmp11.z = (uniforms.f[5].zzzz + reg_tmp11.zzzz).z;
    return false;
}
bool sub_71_94() {
    // 71: flr
    reg_tmp13.x = (floor(reg_tmp0.xxxx)).x;
    // 72: add
    reg_tmp13.x = (reg_tmp0.xxxx + -reg_tmp13).x;
    // 73: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp13.xxxx));
    // 74: ifc
    if (conditional_code.x) {
        sub_75_80();
    } else {
        sub_80_93();
    }
    // 93: nop
    return false;
}
bool sub_75_80() {
    // 75: mov
    reg_tmp12.zw = (uniforms.f[5].xxxy).zw;
    // 76: dp4
    reg_tmp11.x = dot(mul_safe(uniforms.f[32 + address_registers.y].wzyx, reg_tmp12), vec4(1.0));
    // 77: dp4
    reg_tmp11.y = dot(mul_safe(uniforms.f[33 + address_registers.y].wzyx, reg_tmp12), vec4(1.0));
    // 78: mul
    reg_tmp11.xy = (mul_safe(reg_tmp11.xyyy, reg_tmp14.zwww)).xy;
    // 79: add
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_80_93() {
    // 80: mov
    reg_tmp14 = uniforms.f[32 + address_registers.y].wzyx;
    // 81: mov
    reg_tmp13 = uniforms.f[33 + address_registers.y].wzyx;
    // 82: cmp
    conditional_code = notEqual(vec2(uniforms.f[5].xxxx), vec2(vs_in_reg0.zwww));
    // 83: ifc
    if (!conditional_code.y) {
        sub_84_88();
    } else {
        sub_88_92();
    }
    // 92: nop
    return false;
}
bool sub_84_88() {
    // 84: ifc
    if (!conditional_code.x) {
        sub_85_86();
    } else {
        sub_86_87();
    }
    // 87: nop
    return false;
}
bool sub_85_86() {
    // 85: mov
    reg_tmp11.xy = (reg_tmp14.xyyy).xy;
    return false;
}
bool sub_86_87() {
    // 86: mov
    reg_tmp11.xy = (reg_tmp13.zwww).xy;
    return false;
}
bool sub_88_92() {
    // 88: ifc
    if (!conditional_code.x) {
        sub_89_90();
    } else {
        sub_90_91();
    }
    // 91: nop
    return false;
}
bool sub_89_90() {
    // 89: mov
    reg_tmp11.xy = (reg_tmp13.xyyy).xy;
    return false;
}
bool sub_90_91() {
    // 90: mov
    reg_tmp11.xy = (reg_tmp14.zwww).xy;
    return false;
}
bool sub_94_96() {
    // 94: dp4
    reg_tmp11.x = dot(mul_safe(uniforms.f[32 + address_registers.y].wzyx, reg_tmp1), vec4(1.0));
    // 95: dp4
    reg_tmp11.y = dot(mul_safe(uniforms.f[33 + address_registers.y].wzyx, reg_tmp1), vec4(1.0));
    return false;
}
bool sub_98_134() {
    uint jmp_to = 98u;
    while (true) {
        switch (jmp_to) {
        case 98u: {
            // 98: jmpu
            if (!uniforms.b[8]) {
                { jmp_to = 124u; break; }
            }
            // 99: dp4
            reg_tmp3.x = dot(mul_safe(uniforms.f[32 + address_registers.x].wzyx, reg_tmp1), vec4(1.0));
            // 100: dp4
            reg_tmp3.y = dot(mul_safe(uniforms.f[33 + address_registers.x].wzyx, reg_tmp1), vec4(1.0));
            // 101: dp4
            reg_tmp3.z = dot(mul_safe(uniforms.f[34 + address_registers.x].wzyx, reg_tmp1), vec4(1.0));
            // 102: mov
            reg_tmp3.w = (reg_tmp1.wwww).w;
            // 103: mov
            reg_tmp11 = uniforms.f[85].wyzz;
            // 104: mov
            reg_tmp11.z = (-uniforms.f[34 + address_registers.x].xxxx).z;
            // 105: cmp
            conditional_code.x = uniforms.f[5].xxxx.x != reg_tmp11.xzzz.x;
            conditional_code.y = uniforms.f[5].xxxx.y < reg_tmp11.xzzz.y;
            // 106: jmpc
            if (any(not(conditional_code))) {
                { jmp_to = 133u; break; }
            }
            // 107: ifu
            if (uniforms.b[9]) {
                sub_108_119();
            } else {
                sub_119_123();
            }
            // 123: jmpu
            if (uniforms.b[8]) {
                { jmp_to = 133u; break; }
            }
        }
        case 124u: {
            // 124: dp4
            reg_tmp3.x = dot(mul_safe(uniforms.f[32 + address_registers.x].wzyx, reg_tmp1), vec4(1.0));
            // 125: dp4
            reg_tmp3.y = dot(mul_safe(uniforms.f[33 + address_registers.x].wzyx, reg_tmp1), vec4(1.0));
            // 126: dp4
            reg_tmp3.z = dot(mul_safe(uniforms.f[34 + address_registers.x].wzyx, reg_tmp1), vec4(1.0));
            // 127: mov
            reg_tmp3.w = (reg_tmp1.wwww).w;
            // 128: mov
            reg_tmp11 = reg_tmp3;
            // 129: dp4
            reg_tmp3.x = dot(mul_safe(uniforms.f[90], reg_tmp11), vec4(1.0));
            // 130: dp4
            reg_tmp3.y = dot(mul_safe(uniforms.f[91], reg_tmp11), vec4(1.0));
            // 131: dp4
            reg_tmp3.z = dot(mul_safe(uniforms.f[92], reg_tmp11), vec4(1.0));
            // 132: mov
            reg_tmp3.w = (reg_tmp11.wwww).w;
        }
        case 133u: {
            // 133: nop
        }
        default: return false;
        }
    }
    return false;
}
bool sub_108_119() {
    // 108: rcp
    reg_tmp12.x = rcp_safe(reg_tmp11.zzzz.x);
    // 109: add
    reg_tmp13.x = (-uniforms.f[85].yyyy + reg_tmp11.zzzz).x;
    // 110: mul
    reg_tmp14.x = (mul_safe(uniforms.f[85].wwww, reg_tmp12.xxxx)).x;
    // 111: mul
    reg_tmp15.x = (mul_safe(reg_tmp14.xxxx, reg_tmp13.xxxx)).x;
    // 112: slt
    reg_tmp14 = vec4(lessThan(reg_tmp15.xxxx,-reg_tmp15.xxxx));
    // 113: slt
    reg_tmp13 = vec4(lessThan(-reg_tmp15.xxxx,reg_tmp15.xxxx));
    // 114: add
    reg_tmp12.x = (reg_tmp13 + -reg_tmp14).x;
    // 115: max
    reg_tmp15.x = (max(reg_tmp15.xxxx, -reg_tmp15.xxxx)).x;
    // 116: add
    reg_tmp15.x = (uniforms.f[5].wwww + reg_tmp15.xxxx).x;
    // 117: flr
    reg_tmp15.x = (floor(reg_tmp15.xxxx)).x;
    // 118: mad
    reg_tmp3.x = (fma_safe(reg_tmp15.xxxx, reg_tmp12.xxxx, reg_tmp3.xxxx)).x;
    return false;
}
bool sub_119_123() {
    // 119: rcp
    reg_tmp15.z = rcp_safe(reg_tmp11.zzzz.x);
    // 120: add
    reg_tmp15.y = (-uniforms.f[85].yyyy + reg_tmp11.zzzz).y;
    // 121: mul
    reg_tmp15.x = (mul_safe(uniforms.f[85].wwww, reg_tmp15.zzzz)).x;
    // 122: mad
    reg_tmp3.x = (fma_safe(reg_tmp15.xxxx, reg_tmp15.yyyy, reg_tmp3.xxxx)).x;
    return false;
}
bool sub_134_4096() {
    uint jmp_to = 134u;
    while (true) {
        switch (jmp_to) {
        case 134u: {
            // 134: mova
            address_registers.x = (ivec2(vs_in_reg0.xxxx)).x;
            // 135: mov
            reg_tmp0 = uniforms.f[9 + address_registers.x].wzyx;
            // 136: mov
            reg_tmp1.xy = (vs_in_reg0.zwzw).xy;
            // 137: mov
            reg_tmp1.zw = (uniforms.f[5].xyxy).zw;
            // 138: mova
            address_registers.xy = ivec2(reg_tmp0.xyyy);
            // 139: mov
            reg_tmp2 = uniforms.f[32 + address_registers.y].wzyx;
            // 140: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 221u; break; }
            }
            // 141: mov
            reg_tmp4 = uniforms.f[31 + address_registers.x].wzyx;
            // 142: mad
            reg_tmp1.xy = (fma_safe(reg_tmp1.xyyy, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
            // 143: call
            {
                sub_98_134();
            }
            // 144: ifu
            if (uniforms.b[8]) {
                sub_145_149();
            } else {
                sub_149_153();
            }
            // 153: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[5].yyyy), vec2(reg_tmp0.wwww));
            // 154: ifc
            if (all(conditional_code)) {
                sub_155_157();
            } else {
                sub_157_179();
            }
            // 179: mov
            reg_tmp11.z = (reg_tmp0.zzzz).z;
            // 180: mul
            reg_tmp9 = mul_safe(uniforms.f[5].zyzy, reg_tmp0.zzzz);
            // 181: flr
            reg_tmp9.xy = (floor(reg_tmp9)).xy;
            // 182: add
            reg_tmp9.xy = (reg_tmp9.zwww + -reg_tmp9.xyyy).xy;
            // 183: mul
            reg_tmp9 = mul_safe(uniforms.f[5].zzzz, reg_tmp9);
            // 184: mov
            reg_tmp14 = uniforms.f[6].wzyx;
            // 185: mov
            reg_tmp11.xy = (reg_tmp9.xyyy).xy;
            // 186: call
            {
                sub_0_98();
            }
            // 187: ifu
            if (uniforms.b[1]) {
                sub_188_190();
            }
            // 190: ifu
            if (uniforms.b[2]) {
                sub_191_192();
            }
            // 192: mov
            vs_out_attr2 = reg_tmp11.xyyy;
            // 193: mul
            reg_tmp9 = mul_safe(uniforms.f[5].zyzy, reg_tmp9.xxxx);
            // 194: flr
            reg_tmp9.xy = (floor(reg_tmp9)).xy;
            // 195: add
            reg_tmp9.xy = (reg_tmp9.zwww + -reg_tmp9.xyyy).xy;
            // 196: mul
            reg_tmp9 = mul_safe(uniforms.f[5].zzzz, reg_tmp9);
            // 197: mov
            reg_tmp14 = uniforms.f[7].wzyx;
            // 198: mov
            reg_tmp11.xy = (reg_tmp9.xyyy).xy;
            // 199: call
            {
                sub_0_98();
            }
            // 200: ifu
            if (uniforms.b[3]) {
                sub_201_203();
            }
            // 203: ifu
            if (uniforms.b[4]) {
                sub_204_205();
            }
            // 205: mov
            vs_out_attr3 = reg_tmp11.xyyy;
            // 206: mul
            reg_tmp9 = mul_safe(uniforms.f[5].zyzy, reg_tmp9.xxxx);
            // 207: flr
            reg_tmp9.xy = (floor(reg_tmp9)).xy;
            // 208: add
            reg_tmp9.xy = (reg_tmp9.zwww + -reg_tmp9.xyyy).xy;
            // 209: mul
            reg_tmp9 = mul_safe(uniforms.f[5].zzzz, reg_tmp9);
            // 210: mov
            reg_tmp14 = uniforms.f[8].wzyx;
            // 211: mov
            reg_tmp11.xy = (reg_tmp9.xyyy).xy;
            // 212: call
            {
                sub_0_98();
            }
            // 213: ifu
            if (uniforms.b[5]) {
                sub_214_216();
            }
            // 216: ifu
            if (uniforms.b[6]) {
                sub_217_218();
            }
            // 218: mov
            vs_out_attr4 = reg_tmp11.xyyy;
            // 219: end
            return true;
        }
        case 221u: {
            // 221: mul
            reg_tmp1.xy = (mul_safe(uniforms.f[36 + address_registers.x].wzzz, reg_tmp1.xyyy)).xy;
            // 222: mov
            reg_tmp14 = uniforms.f[35 + address_registers.x].wzyx;
            // 223: mad
            reg_tmp1.xy = (fma_safe(reg_tmp1.xyyy, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
            // 224: mad
            reg_tmp14.x = (fma_safe(reg_tmp14.xxxx, vs_in_reg0.wwww, reg_tmp14.xxxx)).x;
            // 225: add
            reg_tmp1.xy = (uniforms.f[36 + address_registers.x].yxxx + reg_tmp1.xyyy).xy;
            // 226: mad
            reg_tmp11.y = (fma_safe(reg_tmp2.yyyy, uniforms.f[36 + address_registers.x].zzzz, -reg_tmp2.yyyy)).y;
            // 227: add
            reg_tmp1.x = (reg_tmp1.xxxx + reg_tmp14.xxxx).x;
            // 228: add
            reg_tmp1.y = (reg_tmp1.yyyy + reg_tmp11.yyyy).y;
            // 229: call
            {
                sub_98_134();
            }
            // 230: dp4
            vs_out_attr0.x = dot(mul_safe(uniforms.f[86].wzyx, reg_tmp3), vec4(1.0));
            // 231: dp4
            vs_out_attr0.y = dot(mul_safe(uniforms.f[87].wzyx, reg_tmp3), vec4(1.0));
            // 232: dp4
            vs_out_attr0.z = dot(mul_safe(uniforms.f[88].wzyx, reg_tmp3), vec4(1.0));
            // 233: dp4
            vs_out_attr0.w = dot(mul_safe(uniforms.f[89].wzyx, reg_tmp3), vec4(1.0));
            // 234: ifu
            if (uniforms.b[2]) {
                sub_235_244();
            } else {
                sub_244_253();
            }
            // 253: mov
            vs_out_attr2 = reg_tmp9;
            // 254: add
            reg_tmp8 = reg_tmp8 + -reg_tmp7;
            // 255: mov
            vs_out_attr3 = reg_tmp9;
            // 256: mad
            vs_out_attr1 = fma_safe(reg_tmp8, reg_tmp11.yyyy, reg_tmp7);
            // 257: mov
            vs_out_attr4 = reg_tmp9;
            // 258: end
            return true;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_145_149() {
    // 145: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[0].wzyx, reg_tmp3), vec4(1.0));
    // 146: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[1].wzyx, reg_tmp3), vec4(1.0));
    // 147: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[2].wzyx, reg_tmp3), vec4(1.0));
    // 148: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[3].wzyx, reg_tmp3), vec4(1.0));
    return false;
}
bool sub_149_153() {
    // 149: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp3), vec4(1.0));
    // 150: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp3), vec4(1.0));
    // 151: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp3), vec4(1.0));
    // 152: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp3), vec4(1.0));
    return false;
}
bool sub_155_157() {
    // 155: mov
    vs_out_attr1.xyz = (uniforms.f[5].yyyy).xyz;
    // 156: mov
    vs_out_attr1.w = (reg_tmp0.wwww).w;
    return false;
}
bool sub_157_179() {
    // 157: mova
    address_registers.y = (ivec2(reg_tmp0.wwww)).y;
    // 158: mov
    reg_tmp7 = uniforms.f[32 + address_registers.y].wzyx;
    // 159: mov
    reg_tmp8 = uniforms.f[33 + address_registers.y].wzyx;
    // 160: mov
    reg_tmp9 = uniforms.f[34 + address_registers.y].wzyx;
    // 161: mov
    reg_tmp10 = uniforms.f[35 + address_registers.y].wzyx;
    // 162: mov
    reg_tmp11.xy = (vs_in_reg0.zwww).xy;
    // 163: flr
    reg_tmp14.x = (floor(reg_tmp0.yyyy)).x;
    // 164: add
    reg_tmp14.x = (reg_tmp0.yyyy + -reg_tmp14.xxxx).x;
    // 165: cmp
    conditional_code = lessThanEqual(vec2(uniforms.f[5].wwww), vec2(reg_tmp14.xxxx));
    // 166: ifc
    if (conditional_code.x) {
        sub_167_171();
    }
    // 171: max
    reg_tmp11.xy = (max(reg_tmp11.xyyy, -reg_tmp11.xyyy)).xy;
    // 172: add
    reg_tmp8 = reg_tmp8 + -reg_tmp7;
    // 173: mad
    reg_tmp8 = fma_safe(reg_tmp8, reg_tmp11.xxxx, reg_tmp7);
    // 174: add
    reg_tmp10 = reg_tmp10 + -reg_tmp9;
    // 175: mad
    reg_tmp10 = fma_safe(reg_tmp10, reg_tmp11.xxxx, reg_tmp9);
    // 176: add
    reg_tmp10 = reg_tmp10 + -reg_tmp8;
    // 177: mad
    reg_tmp10 = fma_safe(reg_tmp10, reg_tmp11.yyyy, reg_tmp8);
    // 178: mov
    vs_out_attr1 = reg_tmp10;
    return false;
}
bool sub_167_171() {
    // 167: rcp
    reg_tmp11.z = rcp_safe(reg_tmp4.xxxx.x);
    // 168: rcp
    reg_tmp11.w = rcp_safe(reg_tmp4.yyyy.x);
    // 169: add
    reg_tmp11.xy = (reg_tmp1.xyyy + -reg_tmp4.zwww).xy;
    // 170: mul
    reg_tmp11.xy = (mul_safe(reg_tmp11.xyyy, reg_tmp11.zwww)).xy;
    return false;
}
bool sub_188_190() {
    // 188: mov
    reg_tmp11.xy = (reg_tmp11.yxxx).xy;
    // 189: add
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_191_192() {
    // 191: add
    reg_tmp11.xy = (uniforms.f[5].yyyy + -reg_tmp11.yxxx).xy;
    return false;
}
bool sub_201_203() {
    // 201: mov
    reg_tmp11.xy = (reg_tmp11.yxxx).xy;
    // 202: add
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_204_205() {
    // 204: add
    reg_tmp11.xy = (uniforms.f[5].yyyy + -reg_tmp11.yxxx).xy;
    return false;
}
bool sub_214_216() {
    // 214: mov
    reg_tmp11.xy = (reg_tmp11.yxxx).xy;
    // 215: add
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_217_218() {
    // 217: add
    reg_tmp11.xy = (uniforms.f[5].yyyy + -reg_tmp11.yxxx).xy;
    return false;
}
bool sub_235_244() {
    // 235: max
    reg_tmp11 = max(vs_in_reg0.zwzw, -vs_in_reg0.zwzw);
    // 236: mova
    address_registers.xy = ivec2(reg_tmp0.zxxx);
    // 237: nop
    // 238: mul
    reg_tmp9.xy = (mul_safe(uniforms.f[32 + address_registers.x].yxxx, reg_tmp11)).xy;
    // 239: slti
    reg_tmp11.zw = (vec4(lessThan(reg_tmp11,uniforms.f[5].yyyy))).zw;
    // 240: mov
    reg_tmp7 = uniforms.f[37 + address_registers.y].wzyx;
    // 241: mad
    reg_tmp9.xy = (fma_safe(reg_tmp11.zwww, uniforms.f[32 + address_registers.x].wzzz, reg_tmp9.xyyy)).xy;
    // 242: mov
    reg_tmp8 = uniforms.f[38 + address_registers.y].wzyx;
    // 243: add
    reg_tmp9.y = (uniforms.f[5].yyyy + -reg_tmp9.yyyy).y;
    return false;
}
bool sub_244_253() {
    // 244: max
    reg_tmp11 = max(vs_in_reg0.zwzw, -vs_in_reg0.zwzw);
    // 245: mova
    address_registers.xy = ivec2(reg_tmp0.zwww);
    // 246: nop
    // 247: mul
    reg_tmp9.xy = (mul_safe(uniforms.f[32 + address_registers.x].yxxx, reg_tmp11)).xy;
    // 248: slti
    reg_tmp11.zw = (vec4(lessThan(reg_tmp11,uniforms.f[5].yyyy))).zw;
    // 249: mov
    reg_tmp7 = uniforms.f[32 + address_registers.y].wzyx;
    // 250: mad
    reg_tmp9.xy = (fma_safe(reg_tmp11.zwww, uniforms.f[32 + address_registers.x].wzzz, reg_tmp9.xyyy)).xy;
    // 251: mov
    reg_tmp8 = uniforms.f[33 + address_registers.y].wzyx;
    // 252: add
    reg_tmp9.y = (uniforms.f[5].yyyy + -reg_tmp9.yyyy).y;
    return false;
}
// reference: E62739D3067DBF59, 0797E835195908AB
// shader: 8DD9, 5D764F9A6220D694

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

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];
 in vec4 vs_out_attr4[];

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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);
    texcoord1 = vec2(vtx.attributes[3].x, vtx.attributes[3].y);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);

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
// reference: 5DAD5699F59B3586, 5D764F9A6220D694
// shader: 8B30, 534BFE759D3374EA

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (combiner_buffer.a) + (last_tex_env_out.a) * (1.0 - (combiner_buffer.a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 67DAB4CF030879C8, 534BFE759D3374EA
// program: 0797E835195908AB, 5D764F9A6220D694, 534BFE759D3374EA
// shader: 8B30, 855861C52D7BC062

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (texcolor1.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (texcolor1.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor1.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (combiner_buffer.a) + (last_tex_env_out.a) * (1.0 - (combiner_buffer.a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2F8F67013B79125C, 855861C52D7BC062
// program: 0797E835195908AB, 5D764F9A6220D694, 855861C52D7BC062
// shader: 8B30, 117DEA4BEBB32612

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (texcolor1.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (texcolor1.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor1.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (combiner_buffer.a) + (last_tex_env_out.a) * (1.0 - (combiner_buffer.a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2F8F6701304EE848, 117DEA4BEBB32612
// program: 0797E835195908AB, 5D764F9A6220D694, 117DEA4BEBB32612
// reference: DE8D15F5A253A70E, D09770228B1223B9
// shader: 8B30, 69EB53926B4DD843

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D4F4BEF9903A5081, 69EB53926B4DD843
// program: D09770228B1223B9, 0282066502FA87A6, 69EB53926B4DD843
// reference: A8761BA4B4A619F0, 05D8F71C78F19800
// reference: CF134005B4A619F0, 4C2527A078F19800
// reference: D4F4BEF996D7068D, 47C12CB4EFB88BF9
// reference: 62769A2532C82302, 077C1C336144D3DF
// reference: B7261CD4AB219FAD, B038D2DA51D280D0
// reference: 754186C8CF0D7939, 0F6657B9E57FE0A0
// reference: 10CCADFCAFE5CE2E, FF0287BA9B67882F
// reference: 0A45E4E6AB219FAD, B038D2DABE700107
// reference: D4F4BEF9C2862184, 4982B44D494E20C9
// reference: 62769A258CF01A24, 273644D90ADF2711
// reference: 3AFDFBAD1519A68B, 9F4472EE66D05307
// reference: 1AF23575D701683E, 124DD61C45CF942F
// reference: 1080747781DE1D70, C59EA4758EB9FB8A
// reference: 10807477C4C04310, BE4C56FF608580E6
// reference: 108074777A3E6CC9, BE4C56FF608580E6
// reference: ABB514A34E237FCD, E7E676F465F0504F
// reference: 07FFBB629E642EDD, 5C7B31B1941AC301
// reference: 4A3353029F171DA1, EA5E28DA420D4997
// reference: FB067E7F4E237FCD, E7E676F465F0504F
// reference: 10807477764CB4FC, 608860C78256AB9B
// reference: 77E52FD6764CB4FC, 5121C8718256AB9B
// reference: 58D5A7B9A8286751, 327A57EE33BE2826
// reference: 682266BFC8E52086, 891DC8E84DC6467D
// reference: 0F473D1EC8E52086, FA0E09FA4DC6467D
// shader: 8B31, 8AE17B67DBD7446D

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
layout(location = 7) in vec4 vs_in_reg7;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;
 out vec4 vs_out_attr4;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr4 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_7_78();
bool sub_15_25();
bool sub_25_33();
bool sub_34_46();
bool sub_46_71();
bool sub_47_65();
bool sub_65_70();
bool sub_78_95();
bool sub_95_171();
bool sub_134_154();
bool sub_135_140();
bool sub_140_153();
bool sub_144_147();
bool sub_147_152();
bool sub_154_167();
bool sub_155_160();
bool sub_160_166();
bool sub_171_181();
bool sub_176_180();
bool sub_177_178();
bool sub_181_209();
bool sub_183_189();
bool sub_189_208();
bool sub_192_198();
bool sub_198_207();
bool sub_199_203();
bool sub_203_206();
bool sub_209_216();
bool sub_216_220();
bool sub_267_4096();

bool exec_shader() {
    sub_267_4096();
    return true;
}

bool sub_7_78() {
    // 7: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 8: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 9: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 10: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 11: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 12: mov
    reg_tmp0 = uniforms.f[7];
    // 13: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 14: ifu
    if (uniforms.b[2]) {
        sub_15_25();
    } else {
        sub_25_33();
    }
    // 33: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_34_46();
    } else {
        sub_46_71();
    }
    // 71: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 72: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 73: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 74: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 75: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 76: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 77: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_15_25() {
    // 15: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 16: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 17: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 18: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 19: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 20: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 21: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 22: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 23: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 24: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_25_33() {
    // 25: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22], reg_tmp15), vec4(1.0));
    // 26: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23], reg_tmp15), vec4(1.0));
    // 27: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24], reg_tmp15), vec4(1.0));
    // 28: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 29: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 30: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 31: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 32: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_34_46() {
    // 34: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 35: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 36: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 37: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 38: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 39: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 40: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 41: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 42: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 43: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 44: call
    {
        sub_78_95();
    }
    // 45: nop
    return false;
}
bool sub_46_71() {
    // 46: ifc
    if (all(conditional_code)) {
        sub_47_65();
    } else {
        sub_65_70();
    }
    // 70: nop
    return false;
}
bool sub_47_65() {
    // 47: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 48: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 49: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 50: dp3
    reg_tmp11.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 51: dp3
    reg_tmp11.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 52: dp3
    reg_tmp11.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 53: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 54: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 55: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 56: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 57: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 58: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 59: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 60: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 61: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 62: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 63: call
    {
        sub_95_171();
    }
    // 64: nop
    return false;
}
bool sub_65_70() {
    // 65: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 66: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 67: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 68: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 69: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_78_95() {
    uint jmp_to = 78u;
    while (true) {
        switch (jmp_to) {
        case 78u: {
            // 78: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 79: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 80: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 81: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 82: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 83: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 84: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 85: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 94u; break; }
            }
            // 86: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 87: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 88: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 89: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 90: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 91: jmpc
            if (conditional_code.x) {
                { jmp_to = 94u; break; }
            }
            // 92: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 93: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 94u: {
            // 94: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_95_171() {
    uint jmp_to = 95u;
    while (true) {
        switch (jmp_to) {
        case 95u: {
            // 95: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 96: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 97: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 98: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 99: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 100: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 101: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 102: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 103: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 104: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 170u; break; }
            }
            // 105: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 106: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 107: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 108: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 109: dp3
            reg_tmp5.w = dot(vec3(mul_safe(reg_tmp5, reg_tmp5)), vec3(1.0));
            // 110: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 111: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 112: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 113: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 114: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 115: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 116: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 117: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 118: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 119: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 120: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 121: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 122: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 123: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 124: jmpc
            if (!conditional_code.x) {
                { jmp_to = 132u; break; }
            }
            // 125: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 126: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 127: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 128: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp7, reg_tmp7), vec4(1.0)));
            // 129: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 130: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 131: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 170u; break; }
            }
        }
        case 132u: {
            // 132: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 133: ifc
            if (conditional_code.x) {
                sub_134_154();
            } else {
                sub_154_167();
            }
            // 167: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp8, reg_tmp8), vec4(1.0)));
            // 168: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 169: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 170u: {
            // 170: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_134_154() {
    // 134: ifc
    if (conditional_code.y) {
        sub_135_140();
    } else {
        sub_140_153();
    }
    // 153: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_135_140() {
    // 135: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 136: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 137: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 138: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 139: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_140_153() {
    // 140: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 141: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 142: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 143: ifc
    if (conditional_code.x) {
        sub_144_147();
    } else {
        sub_147_152();
    }
    // 152: nop
    return false;
}
bool sub_144_147() {
    // 144: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 145: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 146: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_147_152() {
    // 147: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 148: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 149: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 150: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 151: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_154_167() {
    // 154: ifc
    if (conditional_code.y) {
        sub_155_160();
    } else {
        sub_160_166();
    }
    // 166: nop
    return false;
}
bool sub_155_160() {
    // 155: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 156: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 157: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 158: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 159: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_160_166() {
    // 160: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 161: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 162: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 163: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 164: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 165: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_171_181() {
    // 171: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 172: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 173: mov
    reg_tmp9 = uniforms.f[21];
    // 174: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 175: ifc
    if (conditional_code.y) {
        sub_176_180();
    }
    // 180: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_176_180() {
    // 176: ifu
    if (uniforms.b[7]) {
        sub_177_178();
    }
    // 178: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 179: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_177_178() {
    // 177: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_181_209() {
    // 181: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 182: ifu
    if (uniforms.b[9]) {
        sub_183_189();
    } else {
        sub_189_208();
    }
    // 208: nop
    return false;
}
bool sub_183_189() {
    // 183: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    // 184: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 185: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11].xywz, reg_tmp6), vec4(1.0));
    // 186: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12].xywz, reg_tmp6), vec4(1.0));
    // 187: mov
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    // 188: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_189_208() {
    // 189: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 190: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 191: ifc
    if (all(not(conditional_code))) {
        sub_192_198();
    } else {
        sub_198_207();
    }
    // 207: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_192_198() {
    // 192: mov
    reg_tmp6 = reg_tmp10;
    // 193: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11], reg_tmp6), vec4(1.0));
    // 194: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12], reg_tmp6), vec4(1.0));
    // 195: dp4
    reg_tmp3.z = dot(mul_safe(uniforms.f[13], reg_tmp6), vec4(1.0));
    // 196: mul
    reg_tmp0.xy = (mul_safe(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 197: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_198_207() {
    // 198: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_199_203();
    } else {
        sub_203_206();
    }
    // 206: nop
    return false;
}
bool sub_199_203() {
    // 199: call
    {
        sub_209_216();
    }
    // 200: dp3
    reg_tmp3.x = dot(vec3(mul_safe(uniforms.f[11], reg_tmp6)), vec3(1.0));
    // 201: dp3
    reg_tmp3.y = dot(vec3(mul_safe(uniforms.f[12], reg_tmp6)), vec3(1.0));
    // 202: dp3
    reg_tmp3.z = dot(vec3(mul_safe(uniforms.f[13], reg_tmp6)), vec3(1.0));
    return false;
}
bool sub_203_206() {
    // 203: call
    {
        sub_216_220();
    }
    // 204: dp4
    reg_tmp3.x = dot(mul_safe(uniforms.f[11], reg_tmp6), vec4(1.0));
    // 205: dp4
    reg_tmp3.y = dot(mul_safe(uniforms.f[12], reg_tmp6), vec4(1.0));
    return false;
}
bool sub_209_216() {
    // 209: mov
    reg_tmp2 = -reg_tmp15;
    // 210: dp3
    reg_tmp2.w = dot(vec3(mul_safe(reg_tmp2, reg_tmp2)), vec3(1.0));
    // 211: rsq
    reg_tmp2.w = rsq_safe(reg_tmp2.wwww.x);
    // 212: mul
    reg_tmp2 = mul_safe(reg_tmp2, reg_tmp2.wwww);
    // 213: dp3
    reg_tmp1 = vec4(dot(vec3(mul_safe(reg_tmp2, reg_tmp14)), vec3(1.0)));
    // 214: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 215: mad
    reg_tmp6 = fma_safe(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_216_220() {
    // 216: mov
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    // 217: mov
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    // 218: mad
    reg_tmp6 = fma_safe(reg_tmp14, reg_tmp1, reg_tmp1);
    // 219: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_267_4096() {
    // 267: call
    {
        sub_7_78();
    }
    // 268: call
    {
        sub_171_181();
    }
    // 269: call
    {
        sub_181_209();
    }
    // 270: end
    return true;
}
// reference: CF43A795C7E74B88, 8AE17B67DBD7446D
// shader: 8B30, BE595F2A0F068639

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D4F4BEF951160B11, BE595F2A0F068639
// program: 8AE17B67DBD7446D, 6CF3F3B70E23AA85, BE595F2A0F068639
// shader: 8B30, 514C6BA77A16CC3F

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 34B85B30460716CC, 514C6BA77A16CC3F
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 514C6BA77A16CC3F
// shader: 8B30, 31E5E38F00DE11A7

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position + view);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17,clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17,clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.bbb) + (texcolor0.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 4.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (last_tex_env_out.bbb) + (texcolor0.rgb) * (vec3(1.0) - (last_tex_env_out.bbb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.rgb) + (texcolor0.rgb) * (vec3(1.0) - (secondary_fragment_color.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa) + (const_color[3].rgb) * (vec3(1.0) - (texcolor0.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DDC215F2ACD626BC, 31E5E38F00DE11A7
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 31E5E38F00DE11A7
// shader: 8B30, 4A71A6CCA3FDC0DB

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (texcolor0.aaa) + (const_color[0].rgb) * (vec3(1.0) - (texcolor0.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 153378E8DBD5F897, 4A71A6CCA3FDC0DB
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 4A71A6CCA3FDC0DB
// shader: 8B30, 02AD7BCFB15AD22C

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D9141C1681524A66, 02AD7BCFB15AD22C
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 02AD7BCFB15AD22C
// shader: 8B30, 503038EDC202C120

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position + view);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17,clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17,clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g) + (texcolor0.g), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb) * (vec3(1.0) - (texcolor0.bbb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) + (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (secondary_fragment_color.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (secondary_fragment_color.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (secondary_fragment_color.r) + (rounded_primary_color.a) * (1.0 - (secondary_fragment_color.r)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].aaa) * (texcolor1.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C55D0C1B10F6F0BF, 503038EDC202C120
// program: 0817F9534CBA084E, 3B3AE026C742C7D5, 503038EDC202C120
// shader: 8B30, 2C94D4D26EA29EEA

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (combiner_buffer.a) + (last_tex_env_out.a) * (1.0 - (combiner_buffer.a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 67DAB4CFAE128BAD, 2C94D4D26EA29EEA
// program: 0797E835195908AB, 5D764F9A6220D694, 2C94D4D26EA29EEA
// reference: 10CCADFC8DE67D3E, FF0287BA9B67882F
// shader: 8B31, EEA8A92796589FB3

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
layout(location = 7) in vec4 vs_in_reg7;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_7_78();
bool sub_15_25();
bool sub_25_33();
bool sub_34_46();
bool sub_46_71();
bool sub_47_65();
bool sub_65_70();
bool sub_78_95();
bool sub_95_171();
bool sub_134_154();
bool sub_135_140();
bool sub_140_153();
bool sub_144_147();
bool sub_147_152();
bool sub_154_167();
bool sub_155_160();
bool sub_160_166();
bool sub_171_181();
bool sub_176_180();
bool sub_177_178();
bool sub_272_4096();

bool exec_shader() {
    sub_272_4096();
    return true;
}

bool sub_7_78() {
    // 7: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 8: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 9: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 10: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 11: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 12: mov
    reg_tmp0 = uniforms.f[7];
    // 13: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 14: ifu
    if (uniforms.b[2]) {
        sub_15_25();
    } else {
        sub_25_33();
    }
    // 33: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_34_46();
    } else {
        sub_46_71();
    }
    // 71: mov
    reg_tmp0.x = (uniforms.f[85].xxxx).x;
    // 72: mul
    reg_tmp0 = mul_safe(uniforms.f[82], reg_tmp0);
    // 73: add
    vs_out_attr2 = -reg_tmp15 + -reg_tmp0;
    // 74: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp15), vec4(1.0));
    // 75: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp15), vec4(1.0));
    // 76: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp15), vec4(1.0));
    // 77: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp15), vec4(1.0));
    return false;
}
bool sub_15_25() {
    // 15: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 16: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 17: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp15), vec4(1.0));
    // 18: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp15), vec4(1.0));
    // 19: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp15), vec4(1.0));
    // 20: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 21: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 22: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 23: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 24: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_25_33() {
    // 25: dp4
    reg_tmp7.x = dot(mul_safe(uniforms.f[22], reg_tmp15), vec4(1.0));
    // 26: dp4
    reg_tmp7.y = dot(mul_safe(uniforms.f[23], reg_tmp15), vec4(1.0));
    // 27: dp4
    reg_tmp7.z = dot(mul_safe(uniforms.f[24], reg_tmp15), vec4(1.0));
    // 28: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 29: dp4
    reg_tmp10.x = dot(mul_safe(uniforms.f[0], reg_tmp7), vec4(1.0));
    // 30: dp4
    reg_tmp10.y = dot(mul_safe(uniforms.f[1], reg_tmp7), vec4(1.0));
    // 31: dp4
    reg_tmp10.z = dot(mul_safe(uniforms.f[2], reg_tmp7), vec4(1.0));
    // 32: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_34_46() {
    // 34: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 35: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 36: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 37: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 38: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 39: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 40: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 41: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 42: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 43: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 44: call
    {
        sub_78_95();
    }
    // 45: nop
    return false;
}
bool sub_46_71() {
    // 46: ifc
    if (all(conditional_code)) {
        sub_47_65();
    } else {
        sub_65_70();
    }
    // 70: nop
    return false;
}
bool sub_47_65() {
    // 47: dp3
    reg_tmp12.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 48: dp3
    reg_tmp12.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 49: dp3
    reg_tmp12.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp14)), vec3(1.0));
    // 50: dp3
    reg_tmp11.x = dot(vec3(mul_safe(uniforms.f[22 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 51: dp3
    reg_tmp11.y = dot(vec3(mul_safe(uniforms.f[23 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 52: dp3
    reg_tmp11.z = dot(vec3(mul_safe(uniforms.f[24 + address_registers.x], reg_tmp13)), vec3(1.0));
    // 53: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 54: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 55: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 56: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 57: dp3
    reg_tmp14.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp12)), vec3(1.0));
    // 58: dp3
    reg_tmp14.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp12)), vec3(1.0));
    // 59: dp3
    reg_tmp14.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp12)), vec3(1.0));
    // 60: dp3
    reg_tmp13.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp11)), vec3(1.0));
    // 61: dp3
    reg_tmp13.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp11)), vec3(1.0));
    // 62: dp3
    reg_tmp13.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp11)), vec3(1.0));
    // 63: call
    {
        sub_95_171();
    }
    // 64: nop
    return false;
}
bool sub_65_70() {
    // 65: dp4
    reg_tmp15.x = dot(mul_safe(uniforms.f[90], reg_tmp10), vec4(1.0));
    // 66: dp4
    reg_tmp15.y = dot(mul_safe(uniforms.f[91], reg_tmp10), vec4(1.0));
    // 67: dp4
    reg_tmp15.z = dot(mul_safe(uniforms.f[92], reg_tmp10), vec4(1.0));
    // 68: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 69: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_78_95() {
    uint jmp_to = 78u;
    while (true) {
        switch (jmp_to) {
        case 78u: {
            // 78: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 79: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 80: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 81: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 82: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 83: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 84: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 85: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 94u; break; }
            }
            // 86: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 87: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 88: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 89: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 90: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 91: jmpc
            if (conditional_code.x) {
                { jmp_to = 94u; break; }
            }
            // 92: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 93: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 94u: {
            // 94: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_95_171() {
    uint jmp_to = 95u;
    while (true) {
        switch (jmp_to) {
        case 95u: {
            // 95: dp3
            reg_tmp6.x = dot(vec3(mul_safe(reg_tmp14, reg_tmp14)), vec3(1.0));
            // 96: dp3
            reg_tmp7.x = dot(vec3(mul_safe(reg_tmp12, reg_tmp12)), vec3(1.0));
            // 97: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 98: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 99: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 100: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 101: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 102: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 103: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 104: jmpu
            if (!uniforms.b[3]) {
                { jmp_to = 170u; break; }
            }
            // 105: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 106: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 107: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 108: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 109: dp3
            reg_tmp5.w = dot(vec3(mul_safe(reg_tmp5, reg_tmp5)), vec3(1.0));
            // 110: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 111: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 112: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 113: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 114: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 115: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 116: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 117: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 118: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 119: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 120: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 121: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 122: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 123: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 124: jmpc
            if (!conditional_code.x) {
                { jmp_to = 132u; break; }
            }
            // 125: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 126: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 127: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 128: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp7, reg_tmp7), vec4(1.0)));
            // 129: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 130: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 131: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 170u; break; }
            }
        }
        case 132u: {
            // 132: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 133: ifc
            if (conditional_code.x) {
                sub_134_154();
            } else {
                sub_154_167();
            }
            // 167: dp4
            reg_tmp6 = vec4(dot(mul_safe(reg_tmp8, reg_tmp8), vec4(1.0)));
            // 168: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 169: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 170u: {
            // 170: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_134_154() {
    // 134: ifc
    if (conditional_code.y) {
        sub_135_140();
    } else {
        sub_140_153();
    }
    // 153: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_135_140() {
    // 135: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 136: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 137: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 138: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 139: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_140_153() {
    // 140: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 141: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 142: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 143: ifc
    if (conditional_code.x) {
        sub_144_147();
    } else {
        sub_147_152();
    }
    // 152: nop
    return false;
}
bool sub_144_147() {
    // 144: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 145: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 146: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_147_152() {
    // 147: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 148: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 149: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 150: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 151: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_154_167() {
    // 154: ifc
    if (conditional_code.y) {
        sub_155_160();
    } else {
        sub_160_166();
    }
    // 166: nop
    return false;
}
bool sub_155_160() {
    // 155: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 156: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 157: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 158: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 159: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_160_166() {
    // 160: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 161: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 162: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 163: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 164: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 165: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_171_181() {
    // 171: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 172: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 173: mov
    reg_tmp9 = uniforms.f[21];
    // 174: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 175: ifc
    if (conditional_code.y) {
        sub_176_180();
    }
    // 180: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_176_180() {
    // 176: ifu
    if (uniforms.b[7]) {
        sub_177_178();
    }
    // 178: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 179: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_177_178() {
    // 177: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_272_4096() {
    // 272: call
    {
        sub_7_78();
    }
    // 273: call
    {
        sub_171_181();
    }
    // 274: end
    return true;
}
// reference: 54ADBD3FB876D846, EEA8A92796589FB3
// shader: 8B30, 9FD1C54CE9157EF8

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9D3D766912955A0D, 9FD1C54CE9157EF8
// program: EEA8A92796589FB3, 4BD70AD09292A3DA, 9FD1C54CE9157EF8
// reference: E6610ADC1C5EEACF, 8AE17B67DBD7446D
// shader: 8B30, 0DC67EFF9D626013

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9D3D7669148EACD0, 0DC67EFF9D626013
// program: 8AE17B67DBD7446D, 6CF3F3B70E23AA85, 0DC67EFF9D626013
// shader: 8B31, 08928BA7ED9A1144

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

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_0_4();
bool sub_4_8();
bool sub_8_4096();
bool sub_12_13();
bool sub_13_16();
bool sub_26_35();
bool sub_35_60();
bool sub_36_47();
bool sub_47_51();
bool sub_62_63();
bool sub_63_85();

bool exec_shader() {
    sub_8_4096();
    return true;
}

bool sub_0_4() {
    // 0: mov
    reg_tmp2 = reg_tmp1;
    // 1: dp3
    reg_tmp1.x = dot(vec3(mul_safe(uniforms.f[8].xyyy, reg_tmp2.xyzz)), vec3(1.0));
    // 2: dp3
    reg_tmp1.y = dot(vec3(mul_safe(uniforms.f[8].zwww, reg_tmp2.xyzz)), vec3(1.0));
    // 3: add
    reg_tmp1.xy = (uniforms.f[9].xyyy + reg_tmp1.xyyy).xy;
    return false;
}
bool sub_4_8() {
    // 4: mov
    reg_tmp3 = reg_tmp0;
    // 5: mov
    reg_tmp3.z = (uniforms.f[16].zzzz).z;
    // 6: dp3
    reg_tmp0.x = dot(vec3(mul_safe(uniforms.f[8].xyyy, reg_tmp3.xyzz)), vec3(1.0));
    // 7: dp3
    reg_tmp0.y = dot(vec3(mul_safe(uniforms.f[8].zwww, reg_tmp3.xyzz)), vec3(1.0));
    return false;
}
bool sub_8_4096() {
    // 8: mov
    reg_tmp15 = vs_in_reg0;
    // 9: mov
    reg_tmp0.zw = (uniforms.f[16].zwzw).zw;
    // 10: mov
    reg_tmp0.xy = (reg_tmp15.xyyy).xy;
    // 11: ifu
    if (uniforms.b[1]) {
        sub_12_13();
    } else {
        sub_13_16();
    }
    // 16: add
    reg_tmp1.xy = (uniforms.f[7].xyyy + reg_tmp1.xyyy).xy;
    // 17: callu
    if (uniforms.b[2]) {
        sub_0_4();
    }
    // 18: mov
    reg_tmp1.z = (uniforms.f[7].zzzz).z;
    // 19: dphi
    reg_tmp3.x = dot(mul_safe(vec4(reg_tmp1.xyz, 1.0), uniforms.f[0]), vec4(1.0));
    // 20: dphi
    reg_tmp3.y = dot(mul_safe(vec4(reg_tmp1.xyz, 1.0), uniforms.f[1]), vec4(1.0));
    // 21: dphi
    reg_tmp3.z = dot(mul_safe(vec4(reg_tmp1.xyz, 1.0), uniforms.f[2]), vec4(1.0));
    // 22: mova
    address_registers.x = (ivec2(uniforms.f[7].wwww)).x;
    // 23: dphi
    vs_out_attr1.x = dot(mul_safe(vec4(reg_tmp15.zwww.xyz, 1.0), uniforms.f[18 + address_registers.x]), vec4(1.0));
    // 24: dphi
    vs_out_attr1.yzw = vec3(dot(mul_safe(vec4(reg_tmp15.zwww.xyz, 1.0), uniforms.f[19 + address_registers.x]), vec4(1.0)));
    // 25: ifu
    if (uniforms.b[3]) {
        sub_26_35();
    } else {
        sub_35_60();
    }
    // 60: mov
    vs_out_attr3.w = (uniforms.f[16].xxxx).w;
    // 61: ifu
    if (uniforms.b[5]) {
        sub_62_63();
    } else {
        sub_63_85();
    }
    // 85: end
    return true;
}
bool sub_12_13() {
    // 12: mov
    reg_tmp1 = reg_tmp0;
    return false;
}
bool sub_13_16() {
    // 13: dp3
    reg_tmp1.x = dot(vec3(mul_safe(uniforms.f[6].xyyy, reg_tmp0.xyzz)), vec3(1.0));
    // 14: dp3
    reg_tmp1.y = dot(vec3(mul_safe(uniforms.f[6].zwww, reg_tmp0.xyzz)), vec3(1.0));
    // 15: mov
    reg_tmp1.zw = (reg_tmp0.zwzw).zw;
    return false;
}
bool sub_26_35() {
    // 26: mov
    reg_tmp4.w = (reg_tmp0.wwww).w;
    // 27: dphi
    reg_tmp4.x = dot(mul_safe(vec4(reg_tmp3.xyzz.xyz, 1.0), uniforms.f[90]), vec4(1.0));
    // 28: dphi
    reg_tmp4.y = dot(mul_safe(vec4(reg_tmp3.xyzz.xyz, 1.0), uniforms.f[91]), vec4(1.0));
    // 29: dphi
    reg_tmp4.z = dot(mul_safe(vec4(reg_tmp3.xyzz.xyz, 1.0), uniforms.f[92]), vec4(1.0));
    // 30: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp4), vec4(1.0));
    // 31: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp4), vec4(1.0));
    // 32: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp4), vec4(1.0));
    // 33: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp4), vec4(1.0));
    // 34: mov
    vs_out_attr3 = -reg_tmp4;
    return false;
}
bool sub_35_60() {
    // 35: ifu
    if (uniforms.b[4]) {
        sub_36_47();
    } else {
        sub_47_51();
    }
    // 51: mul
    reg_tmp4.yz = (mul_safe(uniforms.f[16].xxyy, reg_tmp3.xyzz)).yz;
    // 52: dphi
    reg_tmp3.x = dot(mul_safe(vec4(reg_tmp4.xyzz.xyz, 1.0), uniforms.f[14]), vec4(1.0));
    // 53: dphi
    reg_tmp3.y = dot(mul_safe(vec4(reg_tmp4.xyzz.xyz, 1.0), uniforms.f[15]), vec4(1.0));
    // 54: dphi
    reg_tmp3.w = dot(mul_safe(vec4(reg_tmp4.xyzz.xyz, 1.0), uniforms.f[89]), vec4(1.0));
    // 55: dphi
    reg_tmp3.z = dot(mul_safe(vec4(reg_tmp4.xyzz.xyz, 1.0), uniforms.f[88]), vec4(1.0));
    // 56: mul
    vs_out_attr0.xy = (mul_safe(reg_tmp3.xyyy, reg_tmp3.wwww)).xy;
    // 57: mov
    vs_out_attr0.zw = (reg_tmp3.zwzw).zw;
    // 58: mov
    vs_out_attr3.xyz = (-reg_tmp4.xyzz).xyz;
    // 59: mov
    vs_out_attr3.w = (uniforms.f[16].xxxx).w;
    return false;
}
bool sub_36_47() {
    // 36: rcp
    reg_tmp13.x = rcp_safe(reg_tmp3.zzzz.x);
    // 37: add
    reg_tmp12.x = (-uniforms.f[85].yyyy + reg_tmp3.zzzz).x;
    // 38: mul
    reg_tmp11.x = (mul_safe(uniforms.f[85].wwww, reg_tmp13.xxxx)).x;
    // 39: mul
    reg_tmp14.x = (mul_safe(reg_tmp11.xxxx, reg_tmp12.xxxx)).x;
    // 40: slt
    reg_tmp11 = vec4(lessThan(reg_tmp14.xxxx,-reg_tmp14.xxxx));
    // 41: slt
    reg_tmp12 = vec4(lessThan(-reg_tmp14.xxxx,reg_tmp14.xxxx));
    // 42: add
    reg_tmp13.x = (reg_tmp12 + -reg_tmp11).x;
    // 43: max
    reg_tmp14.x = (max(reg_tmp14.xxxx, -reg_tmp14.xxxx)).x;
    // 44: add
    reg_tmp14.x = (uniforms.f[17].xxxx + reg_tmp14.xxxx).x;
    // 45: flr
    reg_tmp14.x = (floor(reg_tmp14.xxxx)).x;
    // 46: mad
    reg_tmp4.x = (fma_safe(reg_tmp14.xxxx, reg_tmp13.xxxx, reg_tmp3.xxxx)).x;
    return false;
}
bool sub_47_51() {
    // 47: rcp
    reg_tmp13.x = rcp_safe(reg_tmp3.zzzz.x);
    // 48: add
    reg_tmp12.x = (-uniforms.f[85].yyyy + reg_tmp3.zzzz).x;
    // 49: mul
    reg_tmp11.x = (mul_safe(uniforms.f[85].wwww, reg_tmp13.xxxx)).x;
    // 50: mad
    reg_tmp4.x = (fma_safe(reg_tmp11.xxxx, reg_tmp12.xxxx, reg_tmp3.xxxx)).x;
    return false;
}
bool sub_62_63() {
    // 62: mov
    vs_out_attr2 = uniforms.f[16].zzzx;
    return false;
}
bool sub_63_85() {
    uint jmp_to = 63u;
    while (true) {
        switch (jmp_to) {
        case 63u: {
            // 63: mov
            reg_tmp2 = uniforms.f[16].xzzz;
            // 64: mova
            address_registers.x = (ivec2(vs_in_reg1.xxxx)).x;
            // 65: mov
            reg_tmp0 = uniforms.f[10 + address_registers.x];
            // 66: callu
            if (uniforms.b[2]) {
                sub_4_8();
            }
            // 67: dp3
            reg_tmp1.x = dot(vec3(mul_safe(uniforms.f[3], reg_tmp0)), vec3(1.0));
            // 68: dp3
            reg_tmp1.y = dot(vec3(mul_safe(uniforms.f[4], reg_tmp0)), vec3(1.0));
            // 69: dp3
            reg_tmp1.z = dot(vec3(mul_safe(uniforms.f[5], reg_tmp0)), vec3(1.0));
            // 70: dp3
            reg_tmp0.x = dot(vec3(mul_safe(uniforms.f[90], reg_tmp1)), vec3(1.0));
            // 71: dp3
            reg_tmp0.y = dot(vec3(mul_safe(uniforms.f[91], reg_tmp1)), vec3(1.0));
            // 72: dp3
            reg_tmp0.z = dot(vec3(mul_safe(uniforms.f[92], reg_tmp1)), vec3(1.0));
            // 73: dp3
            reg_tmp3.x = dot(vec3(mul_safe(reg_tmp0, reg_tmp0)), vec3(1.0));
            // 74: rsq
            reg_tmp3.x = rsq_safe(reg_tmp3.xxxx.x);
            // 75: mul
            reg_tmp0.xyz = (mul_safe(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
            // 76: add
            reg_tmp4 = uniforms.f[16].xxxx + reg_tmp0.zzzz;
            // 77: mul
            reg_tmp4 = mul_safe(uniforms.f[17].xxxx, reg_tmp4);
            // 78: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[16].zzzz), vec2(reg_tmp4.xxxx));
            // 79: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 80: mul
            reg_tmp5 = mul_safe(uniforms.f[17].xxxx, reg_tmp0);
            // 81: jmpc
            if (conditional_code.x) {
                { jmp_to = 84u; break; }
            }
            // 82: rcp
            reg_tmp2.z = rcp_safe(reg_tmp4.xxxx.x);
            // 83: mul
            reg_tmp2.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 84u: {
            // 84: mov
            vs_out_attr2 = reg_tmp2;
        }
        default: return false;
        }
    }
    return false;
}
// reference: CC23BC0AEC3AE2A9, 08928BA7ED9A1144
// shader: 8DD9, 985EF155AF98A9BC

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

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];

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
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);
    texcoord1 = vec2(0.0, 0.0);

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
// reference: 68421C109A4ABEA4, 985EF155AF98A9BC
// shader: 8B30, 6DA54F6020316030

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position + view);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17,clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17,clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((secondary_fragment_color.rgb) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A871B32522CF29D3, 6DA54F6020316030
// program: 08928BA7ED9A1144, 985EF155AF98A9BC, 6DA54F6020316030
// reference: 553CDED325779341, 08928BA7ED9A1144
// shader: 8B30, E8F41C04B52B8455

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((secondary_fragment_color.rgb) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8C191B1A8463649E, E8F41C04B52B8455
// program: 08928BA7ED9A1144, 985EF155AF98A9BC, E8F41C04B52B8455
// reference: FB067E7F275A74A8, E7E676F465F0504F
// reference: 95EE66C710F6F0BF, 503038EDC202C120
// shader: 8B30, 665EE456131D3E7B

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
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
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C5B631207685FA20, 665EE456131D3E7B
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 665EE456131D3E7B
// reference: 95055BFC1FFCF145, 665EE456131D3E7B
// reference: 95055BFC3DFF4255, 665EE456131D3E7B
// shader: 8B31, A2F801F822CFD5CB

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

#define fma_safe(x, y, z) mix(fma(x, y, z), vec4(0.0), isnan(fma(x, y, z)))
#define mul_safe(x, y) mix(x * y, vec4(0.0), isnan(x * y))
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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

bool sub_87_4096();
bool sub_92_103();
bool sub_103_118();

bool exec_shader() {
    sub_87_4096();
    return true;
}

bool sub_87_4096() {
    // 87: dphi
    reg_tmp13.x = dot(mul_safe(vec4(vs_in_reg0.xyz, 1.0), uniforms.f[0]), vec4(1.0));
    // 88: dphi
    reg_tmp13.y = dot(mul_safe(vec4(vs_in_reg0.xyz, 1.0), uniforms.f[1]), vec4(1.0));
    // 89: dphi
    reg_tmp13.z = dot(mul_safe(vec4(vs_in_reg0.xyz, 1.0), uniforms.f[2]), vec4(1.0));
    // 90: mov
    reg_tmp13.w = (vs_in_reg0.wwww).w;
    // 91: ifu
    if (uniforms.b[3]) {
        sub_92_103();
    } else {
        sub_103_118();
    }
    // 118: end
    return true;
}
bool sub_92_103() {
    // 92: mul
    vs_out_attr1 = mul_safe(uniforms.f[8].xxxx, vs_in_reg1);
    // 93: dphi
    reg_tmp12.x = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[90]), vec4(1.0));
    // 94: dphi
    reg_tmp12.y = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[91]), vec4(1.0));
    // 95: dphi
    reg_tmp12.z = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[92]), vec4(1.0));
    // 96: mov
    reg_tmp12.w = (reg_tmp13.wwww).w;
    // 97: dphi
    vs_out_attr2.x = dot(mul_safe(vec4(vs_in_reg2.xyz, 1.0), uniforms.f[3]), vec4(1.0));
    // 98: dphi
    vs_out_attr2.yzw = vec3(dot(mul_safe(vec4(vs_in_reg2.xyz, 1.0), uniforms.f[4]), vec4(1.0)));
    // 99: dp4
    vs_out_attr0.x = dot(mul_safe(uniforms.f[86], reg_tmp12), vec4(1.0));
    // 100: dp4
    vs_out_attr0.y = dot(mul_safe(uniforms.f[87], reg_tmp12), vec4(1.0));
    // 101: dp4
    vs_out_attr0.z = dot(mul_safe(uniforms.f[88], reg_tmp12), vec4(1.0));
    // 102: dp4
    vs_out_attr0.w = dot(mul_safe(uniforms.f[89], reg_tmp12), vec4(1.0));
    return false;
}
bool sub_103_118() {
    // 103: rcp
    reg_tmp11.x = rcp_safe(reg_tmp13.zzzz.x);
    // 104: add
    reg_tmp10.x = (-uniforms.f[85].yyyy + reg_tmp13.zzzz).x;
    // 105: mul
    reg_tmp9.x = (mul_safe(uniforms.f[85].wwww, reg_tmp11.xxxx)).x;
    // 106: mul
    reg_tmp12.x = (mul_safe(reg_tmp9.xxxx, reg_tmp10.xxxx)).x;
    // 107: mov
    reg_tmp13.z = (-reg_tmp13.zzzz).z;
    // 108: add
    reg_tmp13.x = (reg_tmp13.xxxx + reg_tmp12.xxxx).x;
    // 109: dphi
    vs_out_attr2.x = dot(mul_safe(vec4(vs_in_reg2.xyz, 1.0), uniforms.f[3]), vec4(1.0));
    // 110: dphi
    vs_out_attr2.yzw = vec3(dot(mul_safe(vec4(vs_in_reg2.xyz, 1.0), uniforms.f[4]), vec4(1.0)));
    // 111: dphi
    reg_tmp12.w = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[89]), vec4(1.0));
    // 112: dphi
    reg_tmp12.z = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[88]), vec4(1.0));
    // 113: dphi
    reg_tmp12.x = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[5]), vec4(1.0));
    // 114: dphi
    reg_tmp12.y = dot(mul_safe(vec4(reg_tmp13.xyz, 1.0), uniforms.f[6]), vec4(1.0));
    // 115: mul
    vs_out_attr1 = mul_safe(uniforms.f[8].xxxx, vs_in_reg1);
    // 116: mov
    vs_out_attr0.zw = (reg_tmp12).zw;
    // 117: mul
    vs_out_attr0.xy = (mul_safe(reg_tmp12.xyyy, reg_tmp12.wwww)).xy;
    return false;
}
// reference: D095E4993EBE50D9, A2F801F822CFD5CB
// shader: 8DD9, 4E5317772B5CE683

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
// reference: 7B07DA3E334A19B0, 4E5317772B5CE683
// shader: 8B30, 269A87C9F23B0C77

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1.0) - (texcolor0.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((const_color[2].a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) + (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AF123A3AE5CE99F0, 269A87C9F23B0C77
// program: A2F801F822CFD5CB, 4E5317772B5CE683, 269A87C9F23B0C77
// reference: 553CDED3B9C38DAC, 08928BA7ED9A1144
// reference: ABB514A3275A74A8, E7E676F465F0504F
// reference: 95055BFC7685FA20, 665EE456131D3E7B
// reference: DDC215F2FC654C60, 31E5E38F00DE11A7
// shader: 8B30, BE160F66CE9534A1

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) + (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9619EFAFEC7BF4B2, BE160F66CE9534A1
// program: A2F801F822CFD5CB, 4E5317772B5CE683, BE160F66CE9534A1
// reference: C55D0C1B40459A63, 503038EDC202C120
// reference: B89A8B073EBE50D9, A2F801F822CFD5CB
// shader: 8B30, 7EF4D0D86CC23244

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1.0) - (texcolor0.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) + (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AF123A3A91456518, 7EF4D0D86CC23244
// program: A2F801F822CFD5CB, 4E5317772B5CE683, 7EF4D0D86CC23244
// reference: FB067E7F1E901511, E7E676F465F0504F
// reference: C5B6312054864930, 665EE456131D3E7B
// reference: 95055BFC54864930, 665EE456131D3E7B
// shader: 8B30, F4305FF123388BF2

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EFCB983B3302BD10, F4305FF123388BF2
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, F4305FF123388BF2
// shader: 8B30, 60E6F3A5B734F5DC

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3A9B1ECAAAEB01BF, 60E6F3A5B734F5DC
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 60E6F3A5B734F5DC
// shader: 8B30, EC1C5EBA00926296

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1.0) - (texcolor0.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp(min((const_color[2].rgb) + (rounded_primary_color.rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F8FC84D6CEC7E72B, EC1C5EBA00926296
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, EC1C5EBA00926296
// shader: 8B30, EB78BA244684A421

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

float ProcTexLookupLUT(int offset, float coord) {
    coord *= 128.0;
    float index_i = clamp(floor(coord), 0.0, 127.0);
    float index_f = coord - index_i; // fract() cannot be used here because 128.0 needs to be
                                     // extracted as index_i = 127.0 and index_f = 1.0
    vec2 entry = texelFetch(texture_buffer_lut_rg, int(index_i) + offset).rg;
    return clamp(entry.r + entry.g * index_f, 0.0, 1.0);
}
    
int ProcTexNoiseRand1D(int v) {
    const int table[] = int[](0,4,10,8,4,9,7,12,5,15,13,14,11,15,2,11);
    return ((v % 9 + 2) * 3 & 0xF) ^ table[(v / 9) & 0xF];
}

float ProcTexNoiseRand2D(vec2 point) {
    const int table[] = int[](10,2,15,8,0,7,4,5,5,13,2,6,13,9,3,14);
    int u2 = ProcTexNoiseRand1D(int(point.x));
    int v2 = ProcTexNoiseRand1D(int(point.y));
    v2 += ((u2 & 3) == 1) ? 4 : 0;
    v2 ^= (u2 & 1) * 6;
    v2 += 10 + u2;
    v2 &= 0xF;
    v2 ^= table[u2];
    return -1.0 + float(v2) * (2.0 / 15.0);
}

float ProcTexNoiseCoef(vec2 x) {
    vec2 grid  = 9.0 * proctex_noise_f * abs(x + proctex_noise_p);
    vec2 point = floor(grid);
    vec2 frac  = grid - point;

    float g0 = ProcTexNoiseRand2D(point) * (frac.x + frac.y);
    float g1 = ProcTexNoiseRand2D(point + vec2(1.0, 0.0)) * (frac.x + frac.y - 1.0);
    float g2 = ProcTexNoiseRand2D(point + vec2(0.0, 1.0)) * (frac.x + frac.y - 1.0);
    float g3 = ProcTexNoiseRand2D(point + vec2(1.0, 1.0)) * (frac.x + frac.y - 2.0);

    float x_noise = ProcTexLookupLUT(proctex_noise_lut_offset, frac.x);
    float y_noise = ProcTexLookupLUT(proctex_noise_lut_offset, frac.y);
    float x0 = mix(g0, g1, x_noise);
    float x1 = mix(g2, g3, x_noise);
    return mix(x0, x1, y_noise);
}
        vec4 SampleProcTexColor(float lut_coord, int level) {
int lut_width = 128 >> level;
int lut_offsets[8] = int[](0, 128, 192, 224, 0xF0, 0xF8, 0xFC, 0xFE);
int lut_offset = lut_offsets[level];
lut_coord *= float(lut_width - 1);
lut_coord += float(lut_offset);
return texelFetch(texture_buffer_lut_rgba, int(round(lut_coord)) + proctex_lut_offset);
}
vec4 ProcTex() {
vec2 uv = abs(texcoord0);
float u_shift = 0.0;
float v_shift = 0.0;
uv += proctex_noise_a * ProcTexNoiseCoef(uv);
uv = abs(uv);
float u = uv.x + u_shift;
float v = uv.y + v_shift;
u = mix(1.0 - fract(u), fract(u), int(u) % 2 == 0);
v = mix(1.0 - fract(v), fract(v), int(v) % 2 == 0);
float lut_coord = ProcTexLookupLUT(proctex_color_map_offset, ((u + v) * 0.5));
vec4 final_color = SampleProcTexColor(lut_coord, 0);
return final_color;
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor1).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.ggg) * (const_color[0].rgb) + (ProcTex().rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor2.r) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb) - vec3(0.5), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((vec3(1.0) - ProcTex().rrr) * (secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (vec3(1.0) - rounded_primary_color.bbb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9D71AFE28C2CE32C, EB78BA244684A421
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, EB78BA244684A421
// shader: 8B30, 60E6F3A5D7B25A7C

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 87F8E6F8AAEB01BF, 60E6F3A5D7B25A7C
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 60E6F3A5D7B25A7C
// shader: 8B30, 4E6F41EBD74F5E9B

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5949BCE7C34CBF96, 4E6F41EBD74F5E9B
// program: F90A89AAB0E37EE2, 4BD70AD09292A3DA, 4E6F41EBD74F5E9B
// shader: 8B30, 666F2A84AE4BA7E8

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EFCB983B8D3A8436, 666F2A84AE4BA7E8
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 666F2A84AE4BA7E8
// shader: 8B30, 953618BBCDE8201F

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((rounded_primary_color.aaa) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B740F9B314D33899, 953618BBCDE8201F
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 953618BBCDE8201F
// shader: 8B30, 879B17EB8E407719

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((1.0 - texcolor2.g) + (texcolor0.g), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp(min((last_tex_env_out.aaa) + (const_color[1].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 974F376BD6CBF62C, 879B17EB8E407719
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, 879B17EB8E407719
// shader: 8B30, EFCB95C4BA772052

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9D3D766980148362, EFCB95C4BA772052
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, EFCB95C4BA772052
// shader: 8B30, 8525A47DA82E2A53

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9D3D7669C50ADD02, 8525A47DA82E2A53
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 8525A47DA82E2A53
// shader: 8B30, C5B629CA108396DC

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb) + (texcolor2.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp(min((texcolor1.rgb) + (const_color[1].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C78E511C9EDD83B3, C5B629CA108396DC
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, C5B629CA108396DC
// shader: 8B30, CB015D50F51E0299

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 260816BD4FE9E1DF, CB015D50F51E0299
// program: F90A89AAB0E37EE2, 4BD70AD09292A3DA, CB015D50F51E0299
// reference: 9D3D76697BF4F2DB, 8525A47DA82E2A53
// shader: 8B30, 5E0558C22193E3CA

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (rounded_primary_color.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp(min((texcolor0.a) + (texcolor1.g), 1.0) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1.0) - rounded_primary_color.ggg) + (const_color[1].rgb) * (vec3(1.0) - (vec3(1.0) - rounded_primary_color.ggg)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1.0) - rounded_primary_color.bbb) + (const_color[2].rgb) * (vec3(1.0) - (vec3(1.0) - rounded_primary_color.bbb)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8A42B97C9FAEB0CF, 5E0558C22193E3CA
// program: 0817F9534CBA084E, 3B3AE026C742C7D5, 5E0558C22193E3CA
// shader: 8B30, 920CC076B8EA3977

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa) + (const_color[1].rgb) * (vec3(1.0) - (texcolor0.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D568A5A7A9E2F943, 920CC076B8EA3977
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 920CC076B8EA3977
// shader: 8B30, 70CAE6176683B258

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9D3D766977862AEE, 70CAE6176683B258
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 70CAE6176683B258
// shader: 8B30, 1F0EAE546683B258

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FA582DC877862AEE, 1F0EAE546683B258
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 1F0EAE546683B258
// shader: 8B30, 17FCABF53356EF22

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E59F64A1C92FBE94, 17FCABF53356EF22
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 17FCABF53356EF22
// shader: 8B30, 82D84BD93356EF22

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 82FA3F00C92FBE94, 82D84BD93356EF22
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 82D84BD93356EF22
// reference: 9D71AFE2AE2F503C, EB78BA244684A421
// shader: 8B30, C351BAD326CA7F88

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

float ProcTexLookupLUT(int offset, float coord) {
    coord *= 128.0;
    float index_i = clamp(floor(coord), 0.0, 127.0);
    float index_f = coord - index_i; // fract() cannot be used here because 128.0 needs to be
                                     // extracted as index_i = 127.0 and index_f = 1.0
    vec2 entry = texelFetch(texture_buffer_lut_rg, int(index_i) + offset).rg;
    return clamp(entry.r + entry.g * index_f, 0.0, 1.0);
}
    
int ProcTexNoiseRand1D(int v) {
    const int table[] = int[](0,4,10,8,4,9,7,12,5,15,13,14,11,15,2,11);
    return ((v % 9 + 2) * 3 & 0xF) ^ table[(v / 9) & 0xF];
}

float ProcTexNoiseRand2D(vec2 point) {
    const int table[] = int[](10,2,15,8,0,7,4,5,5,13,2,6,13,9,3,14);
    int u2 = ProcTexNoiseRand1D(int(point.x));
    int v2 = ProcTexNoiseRand1D(int(point.y));
    v2 += ((u2 & 3) == 1) ? 4 : 0;
    v2 ^= (u2 & 1) * 6;
    v2 += 10 + u2;
    v2 &= 0xF;
    v2 ^= table[u2];
    return -1.0 + float(v2) * (2.0 / 15.0);
}

float ProcTexNoiseCoef(vec2 x) {
    vec2 grid  = 9.0 * proctex_noise_f * abs(x + proctex_noise_p);
    vec2 point = floor(grid);
    vec2 frac  = grid - point;

    float g0 = ProcTexNoiseRand2D(point) * (frac.x + frac.y);
    float g1 = ProcTexNoiseRand2D(point + vec2(1.0, 0.0)) * (frac.x + frac.y - 1.0);
    float g2 = ProcTexNoiseRand2D(point + vec2(0.0, 1.0)) * (frac.x + frac.y - 1.0);
    float g3 = ProcTexNoiseRand2D(point + vec2(1.0, 1.0)) * (frac.x + frac.y - 2.0);

    float x_noise = ProcTexLookupLUT(proctex_noise_lut_offset, frac.x);
    float y_noise = ProcTexLookupLUT(proctex_noise_lut_offset, frac.y);
    float x0 = mix(g0, g1, x_noise);
    float x1 = mix(g2, g3, x_noise);
    return mix(x0, x1, y_noise);
}
        vec4 SampleProcTexColor(float lut_coord, int level) {
int lut_width = 128 >> level;
int lut_offsets[8] = int[](0, 128, 192, 224, 0xF0, 0xF8, 0xFC, 0xFE);
int lut_offset = lut_offsets[level];
lut_coord *= float(lut_width - 1);
lut_coord += float(lut_offset);
return texelFetch(texture_buffer_lut_rgba, int(round(lut_coord)) + proctex_lut_offset);
}
vec4 ProcTex() {
vec2 uv = abs(texcoord0);
float u_shift = 0.0;
float v_shift = 0.0;
uv += proctex_noise_a * ProcTexNoiseCoef(uv);
uv = abs(uv);
float u = uv.x + u_shift;
float v = uv.y + v_shift;
u = mix(1.0 - fract(u), fract(u), int(u) % 2 == 0);
v = mix(1.0 - fract(v), fract(v), int(v) % 2 == 0);
float lut_coord = ProcTexLookupLUT(proctex_color_map_offset, ((u + v) * 0.5));
vec4 final_color = SampleProcTexColor(lut_coord, 0);
return final_color;
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor1).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[6].position + view);
spot_dir = light_src[6].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[6].diffuse * dot_product) + light_src[6].ambient) * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[6].specular_0) + (light_src[6].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.ggg) * (const_color[0].rgb) + (ProcTex().rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor2.r) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb) - vec3(0.5), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((vec3(1.0) - ProcTex().rrr) * (secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (vec3(1.0) - rounded_primary_color.bbb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B91907DDB17F65F8, C351BAD326CA7F88
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, C351BAD326CA7F88
// shader: 8B30, C7194574792CC15E

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[6].position + view);
spot_dir = light_src[6].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[6].diffuse * dot_product) + light_src[6].ambient) * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[6].specular_0) + (light_src[6].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E1DE991F69D5CFE4, C7194574792CC15E
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, C7194574792CC15E
// shader: 8B30, 98738E8826CD4287

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[6].position + view);
spot_dir = light_src[6].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[6].diffuse * dot_product) + light_src[6].ambient) * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[6].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[6].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((secondary_fragment_color.rgb) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A871B325B930E24A, 98738E8826CD4287
// program: 08928BA7ED9A1144, 985EF155AF98A9BC, 98738E8826CD4287
// reference: E1DE991F4BD67CF4, C7194574792CC15E
// reference: AF5792B622AF7791, C7194574792CC15E
// reference: B91907DD937CD6E8, C351BAD326CA7F88
// reference: B16DF3C34BD67CF4, C7194574792CC15E
// reference: FFE4F86A22AF7791, C7194574792CC15E
// shader: 8B30, 1D1907C5AD82334A

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

float ProcTexLookupLUT(int offset, float coord) {
    coord *= 128.0;
    float index_i = clamp(floor(coord), 0.0, 127.0);
    float index_f = coord - index_i; // fract() cannot be used here because 128.0 needs to be
                                     // extracted as index_i = 127.0 and index_f = 1.0
    vec2 entry = texelFetch(texture_buffer_lut_rg, int(index_i) + offset).rg;
    return clamp(entry.r + entry.g * index_f, 0.0, 1.0);
}
    
int ProcTexNoiseRand1D(int v) {
    const int table[] = int[](0,4,10,8,4,9,7,12,5,15,13,14,11,15,2,11);
    return ((v % 9 + 2) * 3 & 0xF) ^ table[(v / 9) & 0xF];
}

float ProcTexNoiseRand2D(vec2 point) {
    const int table[] = int[](10,2,15,8,0,7,4,5,5,13,2,6,13,9,3,14);
    int u2 = ProcTexNoiseRand1D(int(point.x));
    int v2 = ProcTexNoiseRand1D(int(point.y));
    v2 += ((u2 & 3) == 1) ? 4 : 0;
    v2 ^= (u2 & 1) * 6;
    v2 += 10 + u2;
    v2 &= 0xF;
    v2 ^= table[u2];
    return -1.0 + float(v2) * (2.0 / 15.0);
}

float ProcTexNoiseCoef(vec2 x) {
    vec2 grid  = 9.0 * proctex_noise_f * abs(x + proctex_noise_p);
    vec2 point = floor(grid);
    vec2 frac  = grid - point;

    float g0 = ProcTexNoiseRand2D(point) * (frac.x + frac.y);
    float g1 = ProcTexNoiseRand2D(point + vec2(1.0, 0.0)) * (frac.x + frac.y - 1.0);
    float g2 = ProcTexNoiseRand2D(point + vec2(0.0, 1.0)) * (frac.x + frac.y - 1.0);
    float g3 = ProcTexNoiseRand2D(point + vec2(1.0, 1.0)) * (frac.x + frac.y - 2.0);

    float x_noise = ProcTexLookupLUT(proctex_noise_lut_offset, frac.x);
    float y_noise = ProcTexLookupLUT(proctex_noise_lut_offset, frac.y);
    float x0 = mix(g0, g1, x_noise);
    float x1 = mix(g2, g3, x_noise);
    return mix(x0, x1, y_noise);
}
        vec4 SampleProcTexColor(float lut_coord, int level) {
int lut_width = 128 >> level;
int lut_offsets[8] = int[](0, 128, 192, 224, 0xF0, 0xF8, 0xFC, 0xFE);
int lut_offset = lut_offsets[level];
lut_coord *= float(lut_width - 1);
lut_coord += float(lut_offset);
return texelFetch(texture_buffer_lut_rgba, int(round(lut_coord)) + proctex_lut_offset);
}
vec4 ProcTex() {
vec2 uv = abs(texcoord0);
float u_shift = 0.0;
float v_shift = 0.0;
uv += proctex_noise_a * ProcTexNoiseCoef(uv);
uv = abs(uv);
float u = uv.x + u_shift;
float v = uv.y + v_shift;
u = mix(1.0 - fract(u), fract(u), int(u) % 2 == 0);
v = mix(1.0 - fract(v), fract(v), int(v) % 2 == 0);
float lut_coord = ProcTexLookupLUT(proctex_color_map_offset, ((u + v) * 0.5));
vec4 final_color = SampleProcTexColor(lut_coord, 0);
return final_color;
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor1).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[6].position + view);
spot_dir = light_src[6].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[6].diffuse * dot_product) + light_src[6].ambient) * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[6].specular_0) + (light_src[6].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[7].position + view);
spot_dir = light_src[7].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[7].diffuse * dot_product) + light_src[7].ambient) * LookupLightingLUTUnsigned(23,clamp(light_src[7].dist_atten_scale * length(-view - light_src[7].position) + light_src[7].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[7].specular_0) + (light_src[7].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(23,clamp(light_src[7].dist_atten_scale * length(-view - light_src[7].position) + light_src[7].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.ggg) * (const_color[0].rgb) + (ProcTex().rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor2.r) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb) - vec3(0.5), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((vec3(1.0) - ProcTex().rrr) * (secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (vec3(1.0) - rounded_primary_color.bbb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A1F7AF508309D9E9, 1D1907C5AD82334A
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, 1D1907C5AD82334A
// shader: 8B30, 18E2407606ECF20F

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[6].position + view);
spot_dir = light_src[6].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[6].diffuse * dot_product) + light_src[6].ambient) * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[6].specular_0) + (light_src[6].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[7].position + view);
spot_dir = light_src[7].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[7].diffuse * dot_product) + light_src[7].ambient) * LookupLightingLUTUnsigned(23,clamp(light_src[7].dist_atten_scale * length(-view - light_src[7].position) + light_src[7].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[7].specular_0) + (light_src[7].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(23,clamp(light_src[7].dist_atten_scale * length(-view - light_src[7].position) + light_src[7].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F93031925BA373F5, 18E2407606ECF20F
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 18E2407606ECF20F
// shader: 8B30, 7F90781658C98E76

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[6].position + view);
spot_dir = light_src[6].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[6].diffuse * dot_product) + light_src[6].ambient) * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[6].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[6].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(22,clamp(light_src[6].dist_atten_scale * length(-view - light_src[6].position) + light_src[6].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[7].position + view);
spot_dir = light_src[7].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[7].diffuse * dot_product) + light_src[7].ambient) * LookupLightingLUTUnsigned(23,clamp(light_src[7].dist_atten_scale * length(-view - light_src[7].position) + light_src[7].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[7].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[7].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(23,clamp(light_src[7].dist_atten_scale * length(-view - light_src[7].position) + light_src[7].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((secondary_fragment_color.rgb) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B09F1BA88B465E5B, 7F90781658C98E76
// program: 08928BA7ED9A1144, 985EF155AF98A9BC, 7F90781658C98E76
// reference: F930319279A0C0E5, 18E2407606ECF20F
// reference: B7B93A3B10D9CB80, 18E2407606ECF20F
// reference: A1F7AF50A10A6AF9, 1D1907C5AD82334A
// reference: 5B75373A3D401474, A2F801F822CFD5CB
// reference: 76BB7C614FE9E1DF, CB015D50F51E0299
// reference: B16DF3C369D5CFE4, C7194574792CC15E
// reference: 8B3F3A891FFCF145, 665EE456131D3E7B
// reference: DB8C50551FFCF145, 665EE456131D3E7B
// shader: 8B30, F1F6839C373AD36D

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) - (const_color[1].rrr), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor2.g) - (const_color[1].g), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 2.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - last_tex_env_out.rgb) * (const_color[3].rgb) + (combiner_buffer.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 693A8841A6CB49BC, F1F6839C373AD36D
// program: 4678A4FD452DD0E4, AD7922A63ED54CA7, F1F6839C373AD36D
// reference: 260816BD1F5A8B03, CB015D50F51E0299
// shader: 8B30, 4442E7EA7B008028

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 10807477154432C2, 4442E7EA7B008028
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, 4442E7EA7B008028
// shader: 8B30, ABC43AE6C3CD8C44

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (const_color[0].aaa) + (texcolor0.rgb) * (vec3(1.0) - (const_color[0].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6D916A2065FFD2E3, ABC43AE6C3CD8C44
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, ABC43AE6C3CD8C44
// shader: 8B30, F2F32752324F54DF

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CE312E688DACC545, F2F32752324F54DF
// program: 8809BF824E86BD32, 6CF3F3B70E23AA85, F2F32752324F54DF

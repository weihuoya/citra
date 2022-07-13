// shader: 8B30, FEEC1CF6E866F4C5

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3F41287AC35EF0E8, FEEC1CF6E866F4C5
// program: 0000000000000000, 0000000000000000, FEEC1CF6E866F4C5
// shader: 8B30, 504A73246D7CA46D

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B2FC2A64F1CA3B20, 504A73246D7CA46D
// program: 0000000000000000, 0000000000000000, 504A73246D7CA46D
// shader: 8B31, 9583E5EAA2BC5470

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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
bool sub_9_10();
bool sub_14_15();
bool sub_18_30();
bool sub_20_21();
bool sub_21_25();
bool sub_22_23();
bool sub_23_24();
bool sub_30_40();
bool sub_44_45();
bool sub_45_49();
bool sub_46_47();
bool sub_47_48();
bool sub_56_61();
bool sub_61_70();
bool sub_70_83();
bool sub_83_251();
bool sub_89_189();
bool sub_96_125();
bool sub_107_111();
bool sub_125_184();
bool sub_126_158();
bool sub_137_141();
bool sub_158_183();
bool sub_169_173();
bool sub_189_250();
bool sub_192_202();
bool sub_202_207();
bool sub_208_220();
bool sub_220_245();
bool sub_221_239();
bool sub_239_244();
bool sub_251_268();
bool sub_268_344();
bool sub_307_327();
bool sub_308_313();
bool sub_313_326();
bool sub_317_320();
bool sub_320_325();
bool sub_327_340();
bool sub_328_333();
bool sub_333_339();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    {
        sub_83_251();
    }
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    if (conditional_code.y) {
        sub_7_12();
    }
    conditional_code = equal(uniforms.f[93].xx, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_14_15();
    }
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    if (uniforms.b[9]) {
        sub_18_30();
    } else {
        sub_30_40();
    }
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_44_45();
    } else {
        sub_45_49();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    vs_out_attr5 = reg_tmp4;
    return true;
}
bool sub_7_12() {
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    if (uniforms.b[7]) {
        sub_9_10();
    }
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_9_10() {
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_14_15() {
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_18_30() {
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20_21();
    } else {
        sub_21_25();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_20_21() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_21_25() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22_23();
    } else {
        sub_23_24();
    }
    return false;
}
bool sub_22_23() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_23_24() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_30_40() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp6 = reg_tmp10;
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_44_45() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_45_49() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_46_47();
    } else {
        sub_47_48();
    }
    return false;
}
bool sub_46_47() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_47_48() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_56_61() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_61_70() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_70_83() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp5.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_83_251() {
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    if (uniforms.b[1]) {
        sub_89_189();
    } else {
        sub_189_250();
    }
    return false;
}
bool sub_89_189() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    reg_tmp7 = uniforms.f[93].xxxx;
    reg_tmp12 = uniforms.f[93].xxxx;
    reg_tmp11 = uniforms.f[93].xxxx;
    reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_96_125();
    } else {
        sub_125_184();
    }
    vs_out_attr2 = -reg_tmp15;
    vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
    vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
    vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
    vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_96_125() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_61_70();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_61_70();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_61_70();
    }
    if (uniforms.b[8]) {
        sub_107_111();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_251_268();
    }
    return false;
}
bool sub_107_111() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_61_70();
    }
    return false;
}
bool sub_125_184() {
    if (all(conditional_code)) {
        sub_126_158();
    } else {
        sub_158_183();
    }
    return false;
}
bool sub_126_158() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_70_83();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_70_83();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_70_83();
    }
    if (uniforms.b[8]) {
        sub_137_141();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    {
        sub_268_344();
    }
    return false;
}
bool sub_137_141() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_70_83();
    }
    return false;
}
bool sub_158_183() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_56_61();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_56_61();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_56_61();
    }
    if (uniforms.b[8]) {
        sub_169_173();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_169_173() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_56_61();
    }
    return false;
}
bool sub_189_250() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    if (uniforms.b[2]) {
        sub_192_202();
    } else {
        sub_202_207();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_208_220();
    } else {
        sub_220_245();
    }
    vs_out_attr2 = -reg_tmp15;
    vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
    vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
    vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
    vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_192_202() {
    reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp7.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp7.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp7.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_202_207() {
    address_registers.x = (ivec2(uniforms.f[93].xx)).x;
    reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
    reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
    reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_208_220() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_251_268();
    }
    return false;
}
bool sub_220_245() {
    if (all(conditional_code)) {
        sub_221_239();
    } else {
        sub_239_244();
    }
    return false;
}
bool sub_221_239() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp11.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    {
        sub_268_344();
    }
    return false;
}
bool sub_239_244() {
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_251_268() {
    uint jmp_to = 251u;
    while (true) {
        switch (jmp_to) {
        case 251u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 267u; break; }
            }
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
            conditional_code = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
            if (conditional_code.x) {
                { jmp_to = 267u; break; }
            }
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 267u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_268_344() {
    uint jmp_to = 268u;
    while (true) {
        switch (jmp_to) {
        case 268u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 343u; break; }
            }
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
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
            conditional_code = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            if (!conditional_code.x) {
                { jmp_to = 305u; break; }
            }
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            reg_tmp7.w = (reg_tmp6).w;
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            if (uniforms.b[0]) {
                { jmp_to = 343u; break; }
            }
        }
        case 305u: {
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            if (conditional_code.x) {
                sub_307_327();
            } else {
                sub_327_340();
            }
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 343u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_307_327() {
    if (conditional_code.y) {
        sub_308_313();
    } else {
        sub_313_326();
    }
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_308_313() {
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_313_326() {
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    if (conditional_code.x) {
        sub_317_320();
    } else {
        sub_320_325();
    }
    return false;
}
bool sub_317_320() {
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_320_325() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_327_340() {
    if (conditional_code.y) {
        sub_328_333();
    } else {
        sub_333_339();
    }
    return false;
}
bool sub_328_333() {
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_333_339() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
// reference: DE1D69C0104A708F, 9583E5EAA2BC5470
// shader: 8DD9, D0BDF6740B5CE976

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
// reference: 46A0C2E6B155D5CD, D0BDF6740B5CE976
// shader: 8B30, 2D9A3162586D2ED4

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
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((rounded_primary_color.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B908DBC879ED6C62, 2D9A3162586D2ED4
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 2D9A3162586D2ED4
// reference: D071D0AD10946707, 2D9A3162586D2ED4
// shader: 8B30, 05463B23F6A6DECE

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
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((rounded_primary_color.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5DCCD2B31FBD5830, 05463B23F6A6DECE
// program: 0000000000000000, 0000000000000000, 05463B23F6A6DECE
// reference: E9BBB11479ED6C62, 2D9A3162586D2ED4
// shader: 8B30, 6A43066204BD850B

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1.0) - (texcolor0.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E918A8FA8474610F, 6A43066204BD850B
// program: 0000000000000000, 0000000000000000, 6A43066204BD850B
// reference: 52ACE698104A708F, 9583E5EAA2BC5470
// reference: 9EF8DB0410946707, 2D9A3162586D2ED4
// reference: 1345D91A1FBD5830, 05463B23F6A6DECE
// shader: 8B30, 10ECFB36820EB924

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2584B8EB83D54929, 10ECFB36820EB924
// program: 0000000000000000, 0000000000000000, 10ECFB36820EB924
// reference: 2584B8EB547A6191, 10ECFB36820EB924
// shader: 8B30, 6A430662B5AA6D10

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[0].rgb) * (vec3(1.0) - (texcolor0.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3712860A8474610F, 6A430662B5AA6D10
// program: 0000000000000000, 0000000000000000, 6A430662B5AA6D10
// reference: 1CF819D84806B7BA, 9583E5EAA2BC5470
// shader: 8B30, 5C4A779C86579110

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CF42EAA20782F4B4, 5C4A779C86579110
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 5C4A779C86579110
// reference: CF42EAA275322D78, 5C4A779C86579110
// shader: 8B30, A6DE4066647360B5

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AE4C834531371254, A6DE4066647360B5
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, A6DE4066647360B5
// reference: 9786E2FC584E1931, A6DE4066647360B5
// reference: D071D0AD6224BECB, 2D9A3162586D2ED4
// reference: 9786E2FC2AFEC0FD, A6DE4066647360B5
// reference: 5DCCD2B36D0D81FC, 05463B23F6A6DECE
// shader: 8B30, 3FE461905149667B

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
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[1].a) + (secondary_fragment_color.g) * (1.0 - (1.0 - const_color[1].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (texcolor0.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((rounded_primary_color.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7422FA542D26C35A, 3FE461905149667B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 3FE461905149667B
// shader: 8B30, 38B78B01428358E5

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
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[1].a) + (secondary_fragment_color.g) * (1.0 - (1.0 - const_color[1].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (texcolor0.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((rounded_primary_color.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 19FAFE8DC3BDAD30, 38B78B01428358E5
// program: 0000000000000000, 0000000000000000, 38B78B01428358E5
// reference: 7422FA545F961A96, 3FE461905149667B
// reference: 19FAFE8DB10D74FC, 38B78B01428358E5
// shader: 8B30, DC96D476E5318560

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((rounded_primary_color.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3F32C69ED1F7F15F, DC96D476E5318560
// program: 0000000000000000, 0000000000000000, DC96D476E5318560
// reference: AE4C834530F57863, A6DE4066647360B5
// reference: 9786E2FC2B3CAACA, A6DE4066647360B5
// reference: D071D0AD11560D30, 2D9A3162586D2ED4
// reference: D071D0AD63E6D4FC, 2D9A3162586D2ED4
// shader: 8B31, 60F5295A12916EF0

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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
bool sub_22_27();
bool sub_33_35();
bool sub_35_46();
bool sub_38_39();
bool sub_40_41();
bool sub_42_43();
bool sub_44_45();
bool sub_48_51();
bool sub_52_55();
bool sub_56_59();
bool sub_60_63();
bool sub_65_99();
bool sub_67_86();
bool sub_70_73();
bool sub_74_77();
bool sub_78_81();
bool sub_82_85();
bool sub_88_98();
bool sub_90_93();
bool sub_94_97();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    address_registers.x = (ivec2(vs_in_reg0.xy)).x;
    reg_tmp0 = uniforms.f[6 + address_registers.x].wzyx;
    reg_tmp1.xy = (vs_in_reg0.zwzw).xy;
    reg_tmp1.zw = (uniforms.f[5].xyxy).zw;
    address_registers.xy = ivec2(reg_tmp0.xy);
    reg_tmp2.xw = (uniforms.f[64 + address_registers.y].wwyy).xw;
    reg_tmp2.yz = (uniforms.f[5].xxxx).yz;
    reg_tmp4.x = dot_s(reg_tmp1, reg_tmp2);
    reg_tmp2.yw = (uniforms.f[64 + address_registers.y].zzxx).yw;
    reg_tmp2.xz = (uniforms.f[5].xxxx).xz;
    reg_tmp4.y = dot_s(reg_tmp1, reg_tmp2);
    reg_tmp4.zw = (reg_tmp1.zwzw).zw;
    reg_tmp3.x = dot_s(uniforms.f[32 + address_registers.x].wzyx, reg_tmp4);
    reg_tmp3.y = dot_s(uniforms.f[33 + address_registers.x].wzyx, reg_tmp4);
    reg_tmp3.z = dot_s(uniforms.f[34 + address_registers.x].wzyx, reg_tmp4);
    reg_tmp3.w = (reg_tmp1.wwww).w;
    reg_tmp4.z = (uniforms.f[34 + address_registers.x].xxxx).z;
    reg_tmp4.z = (abs(reg_tmp4.zzzz)).z;
    reg_tmp4.z = (uniforms.f[4].yyyy + reg_tmp4.zzzz).z;
    reg_tmp4.x = (uniforms.f[4].wwww).x;
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp4.xz);
    if (all(conditional_code)) {
        sub_22_27();
    }
    vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp3);
    vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp3);
    vs_out_attr0.z = dot_s(uniforms.f[2].wzyx, reg_tmp3);
    vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp3);
    conditional_code = greaterThanEqual(uniforms.f[5].yy, reg_tmp0.ww);
    if (all(conditional_code)) {
        sub_33_35();
    } else {
        sub_35_46();
    }
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
    if (all(not(conditional_code))) {
        sub_48_51();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_52_55();
    }
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_56_59();
    }
    if (all(conditional_code)) {
        sub_60_63();
    }
    reg_tmp8 = uniforms.f[5].xxxx;
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop64 = 0u; loop64 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop64) {
        sub_65_99();
    }
    vs_out_attr2 = reg_tmp5;
    vs_out_attr3 = reg_tmp6;
    vs_out_attr4 = reg_tmp7;
    return true;
}
bool sub_22_27() {
    reg_tmp4.x = (uniforms.f[4].wwww).x;
    reg_tmp4.y = (-uniforms.f[4].zzzz + reg_tmp4.zzzz).y;
    reg_tmp4.z = rcp_s(reg_tmp4.z);
    reg_tmp4.z = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).z;
    reg_tmp3.x = (fma_s(reg_tmp4.xxxx, reg_tmp4.zzzz, reg_tmp3.xxxx)).x;
    return false;
}
bool sub_33_35() {
    vs_out_attr1.xyz = (uniforms.f[5].yyyy).xyz;
    vs_out_attr1.w = (reg_tmp0.wwww).w;
    return false;
}
bool sub_35_46() {
    address_registers.y = (ivec2(reg_tmp0.ww)).y;
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
    if (all(not(conditional_code))) {
        sub_38_39();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_40_41();
    }
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_42_43();
    }
    if (all(conditional_code)) {
        sub_44_45();
    }
    return false;
}
bool sub_38_39() {
    vs_out_attr1 = uniforms.f[32 + address_registers.y].wzyx;
    return false;
}
bool sub_40_41() {
    vs_out_attr1 = uniforms.f[33 + address_registers.y].wzyx;
    return false;
}
bool sub_42_43() {
    vs_out_attr1 = uniforms.f[34 + address_registers.y].wzyx;
    return false;
}
bool sub_44_45() {
    vs_out_attr1 = uniforms.f[35 + address_registers.y].wzyx;
    return false;
}
bool sub_48_51() {
    reg_tmp5 = uniforms.f[5].xyyy;
    reg_tmp6 = uniforms.f[5].xyyy;
    reg_tmp7 = uniforms.f[5].xyyy;
    return false;
}
bool sub_52_55() {
    reg_tmp5 = uniforms.f[5].yyyy;
    reg_tmp6 = uniforms.f[5].yyyy;
    reg_tmp7 = uniforms.f[5].yyyy;
    return false;
}
bool sub_56_59() {
    reg_tmp5 = uniforms.f[5].xxyy;
    reg_tmp6 = uniforms.f[5].xxyy;
    reg_tmp7 = uniforms.f[5].xxyy;
    return false;
}
bool sub_60_63() {
    reg_tmp5 = uniforms.f[5].yxyy;
    reg_tmp6 = uniforms.f[5].yxyy;
    reg_tmp7 = uniforms.f[5].yxyy;
    return false;
}
bool sub_65_99() {
    conditional_code = equal(uniforms.f[5].yy, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_67_86();
    }
    conditional_code = lessThan(uniforms.f[5].ww, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_88_98();
    }
    reg_tmp8 = uniforms.f[5].yyyy + reg_tmp8;
    return false;
}
bool sub_67_86() {
    address_registers.y = (ivec2(reg_tmp0.zz)).y;
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
    if (all(not(conditional_code))) {
        sub_70_73();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_74_77();
    }
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_78_81();
    }
    if (all(conditional_code)) {
        sub_82_85();
    }
    return false;
}
bool sub_70_73() {
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].wzzz).xy;
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].wzzz).xy;
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].wzzz).xy;
    return false;
}
bool sub_74_77() {
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].yzzz).xy;
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].yzzz).xy;
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].yzzz).xy;
    return false;
}
bool sub_78_81() {
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].wxxx).xy;
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].wxxx).xy;
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].wxxx).xy;
    return false;
}
bool sub_82_85() {
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].yxxx).xy;
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].yxxx).xy;
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].yxxx).xy;
    return false;
}
bool sub_88_98() {
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_90_93();
    }
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_94_97();
    }
    return false;
}
bool sub_90_93() {
    reg_tmp5.xy = (uniforms.f[67 + address_registers.y].yxxx).xy;
    reg_tmp6.xy = (uniforms.f[68 + address_registers.y].yxxx).xy;
    reg_tmp7.xy = (uniforms.f[69 + address_registers.y].yxxx).xy;
    return false;
}
bool sub_94_97() {
    reg_tmp5.xy = (uniforms.f[67 + address_registers.y].wzzz).xy;
    reg_tmp6.xy = (uniforms.f[68 + address_registers.y].wzzz).xy;
    reg_tmp7.xy = (uniforms.f[69 + address_registers.y].wzzz).xy;
    return false;
}
// reference: A0955C54CF6DF4DE, 60F5295A12916EF0
// shader: 8DD9, 42937135801BAA7E

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
// reference: 5DAD5699F59B3586, 42937135801BAA7E
// shader: 8B30, 05D13F9CC9D6B95B

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C036C25FAB94E9C9, 05D13F9CC9D6B95B
// program: 60F5295A12916EF0, 42937135801BAA7E, 05D13F9CC9D6B95B
// reference: EEC1A314972133EB, 60F5295A12916EF0
// shader: 8B30, 8BF922FF6D24063C

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C036C25F5C7A3216, 8BF922FF6D24063C
// program: 0000000000000000, 0000000000000000, 8BF922FF6D24063C
// shader: 8B30, 020972DD34905A71

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EFF8FBA8CAC51E81, 020972DD34905A71
// program: 60F5295A12916EF0, 42937135801BAA7E, 020972DD34905A71
// shader: 8B30, 7FB32728B27320D8

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EFF8FBA83D2BC55E, 7FB32728B27320D8
// program: 0000000000000000, 0000000000000000, 7FB32728B27320D8
// reference: FEFFE999604612BF, A6DE4066647360B5
// shader: 8B30, 49596FF982288015

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6245F9B60FBF0E96, 49596FF982288015
// program: 0000000000000000, 0000000000000000, 49596FF982288015
// shader: 8B30, 8D3E0643E7CCA192

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F578F6D1739B6EEB, 8D3E0643E7CCA192
// program: 0000000000000000, 0000000000000000, 8D3E0643E7CCA192
// shader: 8B30, 19AFA7DEA6CEE421

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 78C5F4CF410FA523, 19AFA7DEA6CEE421
// program: 0000000000000000, 0000000000000000, 19AFA7DEA6CEE421
// shader: 8B30, FA0E7E0AFCC863AD

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9786E2FC16815E35, FA0E7E0AFCC863AD
// program: 0000000000000000, 0000000000000000, FA0E7E0AFCC863AD
// shader: 8B30, 5F8945D019DA9EE2

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1A3BE0E2241595FD, 5F8945D019DA9EE2
// program: 0000000000000000, 0000000000000000, 5F8945D019DA9EE2
// shader: 8B30, 21D833408B259C8A

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 20601F574B3A083B, 21D833408B259C8A
// program: 60F5295A12916EF0, 42937135801BAA7E, 21D833408B259C8A
// shader: 8B30, 4FF5353339EE6CD4

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 20601F5705C28121, 4FF5353339EE6CD4
// program: 60F5295A12916EF0, 42937135801BAA7E, 4FF5353339EE6CD4
// shader: 8B30, C1C789FE58EC10BF

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ADDD1D498E40182C, C1C789FE58EC10BF
// program: 0000000000000000, 0000000000000000, C1C789FE58EC10BF
// shader: 8B30, 3119C0AC5ED36E98

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (primary_fragment_color.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F0811ECAA63DB4E7, 3119C0AC5ED36E98
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 3119C0AC5ED36E98
// shader: 8B30, 3B47F23567C9D426

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DAB3C4832C03C0B1, 3B47F23567C9D426
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 3B47F23567C9D426
// shader: 8B30, 4ACE9F564CD31D6B

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor1.a) * (const_color[1].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 487D8D2DBEC85929, 4ACE9F564CD31D6B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 4ACE9F564CD31D6B
// reference: 6C4AD557D7ECD73C, 4ACE9F564CD31D6B
// reference: C63F3C132CAD7002, 4ACE9F564CD31D6B
// shader: 8B30, 9C9155F74F5A9E32

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 806EDD4B30F57863, 9C9155F74F5A9E32
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 9C9155F74F5A9E32
// shader: 8B30, 46156D30B20FEAE2

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 943A978856FF31EA, 46156D30B20FEAE2
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 46156D30B20FEAE2
// shader: 8B30, B7E1B7F432608C9F

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A4AC234A598C7306, B7E1B7F432608C9F
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, B7E1B7F432608C9F
// shader: 8B30, C4555434E80CC41C

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
vec3 surface_normal = 2.0 * (texcolor1).rgb - 1.0;
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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4FC2F0579565A846, C4555434E80CC41C
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, C4555434E80CC41C
// reference: F72DB75B598C7306, 9C9155F74F5A9E32
// shader: 8B30, ABE439554EDF4B6D

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C40776ED598C7306, ABE439554EDF4B6D
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, ABE439554EDF4B6D
// shader: 8B31, 4E545413B902FBCE

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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
bool sub_9_10();
bool sub_14_15();
bool sub_18_30();
bool sub_20_21();
bool sub_21_25();
bool sub_22_23();
bool sub_23_24();
bool sub_30_40();
bool sub_44_45();
bool sub_45_49();
bool sub_46_47();
bool sub_47_48();
bool sub_56_61();
bool sub_61_70();
bool sub_70_83();
bool sub_83_293();
bool sub_89_210();
bool sub_96_130();
bool sub_107_111();
bool sub_130_199();
bool sub_131_168();
bool sub_142_146();
bool sub_168_198();
bool sub_179_183();
bool sub_208_209();
bool sub_210_292();
bool sub_211_221();
bool sub_221_226();
bool sub_229_246();
bool sub_246_281();
bool sub_247_270();
bool sub_270_280();
bool sub_290_291();
bool sub_293_310();
bool sub_310_386();
bool sub_349_369();
bool sub_350_355();
bool sub_355_368();
bool sub_359_362();
bool sub_362_367();
bool sub_369_382();
bool sub_370_375();
bool sub_375_381();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    {
        sub_83_293();
    }
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    if (conditional_code.y) {
        sub_7_12();
    }
    conditional_code = equal(uniforms.f[93].xx, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_14_15();
    }
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    if (uniforms.b[9]) {
        sub_18_30();
    } else {
        sub_30_40();
    }
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_44_45();
    } else {
        sub_45_49();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    vs_out_attr5 = reg_tmp4;
    return true;
}
bool sub_7_12() {
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    if (uniforms.b[7]) {
        sub_9_10();
    }
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_9_10() {
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_14_15() {
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_18_30() {
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20_21();
    } else {
        sub_21_25();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_20_21() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_21_25() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22_23();
    } else {
        sub_23_24();
    }
    return false;
}
bool sub_22_23() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_23_24() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_30_40() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp6 = reg_tmp10;
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_44_45() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_45_49() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_46_47();
    } else {
        sub_47_48();
    }
    return false;
}
bool sub_46_47() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_47_48() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_56_61() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_61_70() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_70_83() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp5.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_83_293() {
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    if (uniforms.b[1]) {
        sub_89_210();
    } else {
        sub_210_292();
    }
    return false;
}
bool sub_89_210() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    reg_tmp7 = uniforms.f[93].xxxx;
    reg_tmp12 = uniforms.f[93].xxxx;
    reg_tmp11 = uniforms.f[93].xxxx;
    reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_96_130();
    } else {
        sub_130_199();
    }
    vs_out_attr2 = -reg_tmp15;
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_208_209();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_96_130() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_61_70();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_61_70();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_61_70();
    }
    if (uniforms.b[8]) {
        sub_107_111();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp7 = reg_tmp10;
    reg_tmp10.x = dot_s(uniforms.f[22], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[23], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[24], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_293_310();
    }
    return false;
}
bool sub_107_111() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_61_70();
    }
    return false;
}
bool sub_130_199() {
    if (all(conditional_code)) {
        sub_131_168();
    } else {
        sub_168_198();
    }
    return false;
}
bool sub_131_168() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_70_83();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_70_83();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_70_83();
    }
    if (uniforms.b[8]) {
        sub_142_146();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp7 = reg_tmp10;
    reg_tmp10.x = dot_s(uniforms.f[22], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[23], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[24], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    {
        sub_310_386();
    }
    return false;
}
bool sub_142_146() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_70_83();
    }
    return false;
}
bool sub_168_198() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_56_61();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_56_61();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_56_61();
    }
    if (uniforms.b[8]) {
        sub_179_183();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp7 = reg_tmp10;
    reg_tmp10.x = dot_s(uniforms.f[22], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[23], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[24], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_179_183() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_56_61();
    }
    return false;
}
bool sub_208_209() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_210_292() {
    if (uniforms.b[2]) {
        sub_211_221();
    } else {
        sub_221_226();
    }
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_229_246();
    } else {
        sub_246_281();
    }
    vs_out_attr2 = -reg_tmp15;
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_290_291();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_211_221() {
    reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp7.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp7.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp7.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_221_226() {
    address_registers.x = (ivec2(uniforms.f[93].xx)).x;
    reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
    reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
    reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_229_246() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = reg_tmp10;
    reg_tmp10.x = dot_s(uniforms.f[22], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[23], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[24], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_293_310();
    }
    return false;
}
bool sub_246_281() {
    if (all(conditional_code)) {
        sub_247_270();
    } else {
        sub_270_280();
    }
    return false;
}
bool sub_247_270() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp11.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp7 = reg_tmp10;
    reg_tmp10.x = dot_s(uniforms.f[22], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[23], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[24], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    {
        sub_310_386();
    }
    return false;
}
bool sub_270_280() {
    reg_tmp7 = reg_tmp10;
    reg_tmp10.x = dot_s(uniforms.f[22], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[23], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[24], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_290_291() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_293_310() {
    uint jmp_to = 293u;
    while (true) {
        switch (jmp_to) {
        case 293u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 309u; break; }
            }
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
            conditional_code = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
            if (conditional_code.x) {
                { jmp_to = 309u; break; }
            }
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 309u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_310_386() {
    uint jmp_to = 310u;
    while (true) {
        switch (jmp_to) {
        case 310u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 385u; break; }
            }
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
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
            conditional_code = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            if (!conditional_code.x) {
                { jmp_to = 347u; break; }
            }
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            reg_tmp7.w = (reg_tmp6).w;
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            if (uniforms.b[0]) {
                { jmp_to = 385u; break; }
            }
        }
        case 347u: {
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            if (conditional_code.x) {
                sub_349_369();
            } else {
                sub_369_382();
            }
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 385u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_349_369() {
    if (conditional_code.y) {
        sub_350_355();
    } else {
        sub_355_368();
    }
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_350_355() {
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_355_368() {
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    if (conditional_code.x) {
        sub_359_362();
    } else {
        sub_362_367();
    }
    return false;
}
bool sub_359_362() {
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_362_367() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_369_382() {
    if (conditional_code.y) {
        sub_370_375();
    } else {
        sub_375_381();
    }
    return false;
}
bool sub_370_375() {
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_375_381() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
// reference: 146B797CA78570EC, 4E545413B902FBCE
// shader: 8B30, B37B323413853CE0

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
refl_value.r = (lut_scale_rr * LookupLightingLUTSigned(6, dot(light_vector, normal)));
refl_value.g = (lut_scale_rg * LookupLightingLUTSigned(5, dot(light_vector, normal)));
refl_value.b = (lut_scale_rb * LookupLightingLUTSigned(4, dot(light_vector, normal)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (secondary_fragment_color.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor1.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((primary_fragment_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 63CE8E81CD3204B2, B37B323413853CE0
// program: 4E545413B902FBCE, D0BDF6740B5CE976, B37B323413853CE0
// reference: 404B53F6F9C72878, 9583E5EAA2BC5470
// shader: 8B30, FF0C156AA788CC41

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D90FB1F72370825D, FF0C156AA788CC41
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, FF0C156AA788CC41
// shader: 8B30, 783A6618A9DCCFE6

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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((texcolor1.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1DD895623C16DE87, 783A6618A9DCCFE6
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 783A6618A9DCCFE6
// shader: 8B30, FBB5B6DEC0AF5970

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 943A9788BA48DAE9, FBB5B6DEC0AF5970
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, FBB5B6DEC0AF5970
// shader: 8B30, 47C493E4C6F402FD

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
refl_value.r = (lut_scale_rr * LookupLightingLUTSigned(6, dot(light_vector, normal)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = (secondary_fragment_color.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor1.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((primary_fragment_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D290B1ECDA334771, 47C493E4C6F402FD
// program: 4E545413B902FBCE, D0BDF6740B5CE976, 47C493E4C6F402FD
// shader: 8B30, 05EE70C9287E57E8

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 893250922C03C0B1, 05EE70C9287E57E8
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 05EE70C9287E57E8
// shader: 8B31, DD1B5AA99EDA2A36

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_56_61();
bool sub_61_70();
bool sub_70_83();
bool sub_83_251();
bool sub_89_189();
bool sub_96_125();
bool sub_107_111();
bool sub_125_184();
bool sub_126_158();
bool sub_137_141();
bool sub_158_183();
bool sub_169_173();
bool sub_189_250();
bool sub_192_202();
bool sub_202_207();
bool sub_208_220();
bool sub_220_245();
bool sub_221_239();
bool sub_239_244();
bool sub_251_268();
bool sub_268_344();
bool sub_307_327();
bool sub_308_313();
bool sub_313_326();
bool sub_317_320();
bool sub_320_325();
bool sub_327_340();
bool sub_328_333();
bool sub_333_339();
bool sub_344_4096();
bool sub_351_356();
bool sub_353_354();
bool sub_358_359();
bool sub_362_374();
bool sub_364_365();
bool sub_365_369();
bool sub_366_367();
bool sub_367_368();
bool sub_374_384();
bool sub_388_389();
bool sub_389_393();
bool sub_390_391();
bool sub_391_392();
bool sub_401_402();
bool sub_402_406();
bool sub_403_404();
bool sub_404_405();

bool exec_shader() {
    sub_344_4096();
    return true;
}

bool sub_56_61() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_61_70() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_70_83() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp5.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_83_251() {
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    if (uniforms.b[1]) {
        sub_89_189();
    } else {
        sub_189_250();
    }
    return false;
}
bool sub_89_189() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    reg_tmp7 = uniforms.f[93].xxxx;
    reg_tmp12 = uniforms.f[93].xxxx;
    reg_tmp11 = uniforms.f[93].xxxx;
    reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_96_125();
    } else {
        sub_125_184();
    }
    vs_out_attr2 = -reg_tmp15;
    vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
    vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
    vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
    vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_96_125() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_61_70();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_61_70();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_61_70();
    }
    if (uniforms.b[8]) {
        sub_107_111();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_251_268();
    }
    return false;
}
bool sub_107_111() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_61_70();
    }
    return false;
}
bool sub_125_184() {
    if (all(conditional_code)) {
        sub_126_158();
    } else {
        sub_158_183();
    }
    return false;
}
bool sub_126_158() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_70_83();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_70_83();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_70_83();
    }
    if (uniforms.b[8]) {
        sub_137_141();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    {
        sub_268_344();
    }
    return false;
}
bool sub_137_141() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_70_83();
    }
    return false;
}
bool sub_158_183() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_56_61();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_56_61();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_56_61();
    }
    if (uniforms.b[8]) {
        sub_169_173();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_169_173() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_56_61();
    }
    return false;
}
bool sub_189_250() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    if (uniforms.b[2]) {
        sub_192_202();
    } else {
        sub_202_207();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_208_220();
    } else {
        sub_220_245();
    }
    vs_out_attr2 = -reg_tmp15;
    vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
    vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
    vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
    vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_192_202() {
    reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp7.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp7.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp7.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_202_207() {
    address_registers.x = (ivec2(uniforms.f[93].xx)).x;
    reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
    reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
    reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_208_220() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_251_268();
    }
    return false;
}
bool sub_220_245() {
    if (all(conditional_code)) {
        sub_221_239();
    } else {
        sub_239_244();
    }
    return false;
}
bool sub_221_239() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp11.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    {
        sub_268_344();
    }
    return false;
}
bool sub_239_244() {
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_251_268() {
    uint jmp_to = 251u;
    while (true) {
        switch (jmp_to) {
        case 251u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 267u; break; }
            }
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
            conditional_code = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
            if (conditional_code.x) {
                { jmp_to = 267u; break; }
            }
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 267u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_268_344() {
    uint jmp_to = 268u;
    while (true) {
        switch (jmp_to) {
        case 268u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 343u; break; }
            }
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
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
            conditional_code = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            if (!conditional_code.x) {
                { jmp_to = 305u; break; }
            }
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            reg_tmp7.w = (reg_tmp6).w;
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            if (uniforms.b[0]) {
                { jmp_to = 343u; break; }
            }
        }
        case 305u: {
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            if (conditional_code.x) {
                sub_307_327();
            } else {
                sub_327_340();
            }
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 343u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_307_327() {
    if (conditional_code.y) {
        sub_308_313();
    } else {
        sub_313_326();
    }
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_308_313() {
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_313_326() {
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    if (conditional_code.x) {
        sub_317_320();
    } else {
        sub_320_325();
    }
    return false;
}
bool sub_317_320() {
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_320_325() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_327_340() {
    if (conditional_code.y) {
        sub_328_333();
    } else {
        sub_333_339();
    }
    return false;
}
bool sub_328_333() {
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_333_339() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_344_4096() {
    {
        sub_83_251();
    }
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    if (conditional_code.y) {
        sub_351_356();
    }
    conditional_code = equal(uniforms.f[93].xx, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_358_359();
    }
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    if (uniforms.b[9]) {
        sub_362_374();
    } else {
        sub_374_384();
    }
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_388_389();
    } else {
        sub_389_393();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    vs_out_attr5 = reg_tmp4;
    reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_401_402();
    } else {
        sub_402_406();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
    vs_out_attr6 = reg_tmp5;
    return true;
}
bool sub_351_356() {
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    if (uniforms.b[7]) {
        sub_353_354();
    }
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_353_354() {
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_358_359() {
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_362_374() {
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_364_365();
    } else {
        sub_365_369();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_364_365() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_365_369() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_366_367();
    } else {
        sub_367_368();
    }
    return false;
}
bool sub_366_367() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_367_368() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_374_384() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp6 = reg_tmp10;
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_388_389() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_389_393() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_390_391();
    } else {
        sub_391_392();
    }
    return false;
}
bool sub_390_391() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_391_392() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_401_402() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_402_406() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_403_404();
    } else {
        sub_404_405();
    }
    return false;
}
bool sub_403_404() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_404_405() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
// reference: EF0FC1B13841945A, DD1B5AA99EDA2A36
// shader: 8DD9, B80FA1B7F97410CD

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
// reference: FC74FA4ACA1C8C74, B80FA1B7F97410CD
// shader: 8B30, 912778C21CBCCA9C

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor2.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((texcolor2.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 00D052785EEEAB33, 912778C21CBCCA9C
// program: DD1B5AA99EDA2A36, B80FA1B7F97410CD, 912778C21CBCCA9C
// reference: EF0FC1B1D778071B, DD1B5AA99EDA2A36
// reference: 404B53F616FEBB39, 9583E5EAA2BC5470
// shader: 8B30, 412420884B7D7C0F

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(view))));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - secondary_fragment_color.aaa) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.aaa) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (texcolor1.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3F4D457F5FC9F7B0, 412420884B7D7C0F
// program: DD1B5AA99EDA2A36, B80FA1B7F97410CD, 412420884B7D7C0F
// shader: 8B30, DCD1284C69838DDF

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 89325092C0B42BB2, DCD1284C69838DDF
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, DCD1284C69838DDF
// reference: 3A31C41C74FC82FA, 46156D30B20FEAE2
// program: DD1B5AA99EDA2A36, B80FA1B7F97410CD, 46156D30B20FEAE2
// shader: 8B30, D41733ECAC30BB84

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor2.rgb) * (texcolor2.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (secondary_fragment_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FFF7BA5372492619, D41733ECAC30BB84
// program: DD1B5AA99EDA2A36, B80FA1B7F97410CD, D41733ECAC30BB84
// shader: 8B30, 19B8576E9F00D1AF

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (vec3(1.0) - const_color[0].aaa) + (const_color[0].rgb) * (vec3(1.0) - (vec3(1.0) - const_color[0].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (texcolor1.b) + (1.0 - texcolor1.b) * (1.0 - (texcolor1.b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D5E7AAC841A86FE2, 19B8576E9F00D1AF
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 19B8576E9F00D1AF
// shader: 8B30, EBBDAECFBE86663B

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (vec3(1.0) - const_color[0].aaa) + (const_color[0].rgb) * (vec3(1.0) - (vec3(1.0) - const_color[0].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (texcolor1.b) + (1.0 - texcolor1.b) * (1.0 - (texcolor1.b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 86663ED941A86FE2, EBBDAECFBE86663B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, EBBDAECFBE86663B
// shader: 8B30, 05378086BF2F4B98

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (vec3(1.0) - const_color[0].aaa) + (const_color[0].rgb) * (vec3(1.0) - (vec3(1.0) - const_color[0].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (texcolor1.b) + (1.0 - texcolor1.b) * (1.0 - (texcolor1.b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3A925354C96D5899, 05378086BF2F4B98
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 05378086BF2F4B98
// shader: 8B31, 1993E2D62BB91686

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
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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
bool sub_10_28();
bool sub_16_21();
bool sub_18_19();
bool sub_25_26();
bool sub_28_74();
bool sub_32_73();
bool sub_37_39();
bool sub_39_63();
bool sub_42_47();
bool sub_53_62();
bool sub_57_58();
bool sub_58_61();
bool sub_65_72();
bool sub_74_84();
bool sub_81_82();
bool sub_84_111();
bool sub_86_91();
bool sub_91_110();
bool sub_94_100();
bool sub_100_109();
bool sub_101_105();
bool sub_105_108();
bool sub_111_119();
bool sub_113_114();
bool sub_114_118();
bool sub_115_116();
bool sub_116_117();
bool sub_119_126();
bool sub_126_130();
bool sub_130_154();
bool sub_132_136();
bool sub_136_153();
bool sub_137_151();
bool sub_140_147();
bool sub_147_150();
bool sub_151_152();
bool sub_154_170();
bool sub_156_160();
bool sub_160_169();
bool sub_161_167();
bool sub_167_168();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    reg_tmp0 = vs_in_reg0;
    vs_out_attr0 = reg_tmp0;
    vs_out_attr2 = reg_tmp0;
    vs_out_attr1 = reg_tmp0;
    {
        sub_10_28();
    }
    {
        sub_84_111();
    }
    {
        sub_130_154();
    }
    {
        sub_154_170();
    }
    return true;
}
bool sub_10_28() {
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    if (conditional_code.y) {
        sub_16_21();
    }
    if (uniforms.b[12]) {
        sub_28_74();
    }
    if (uniforms.b[5]) {
        sub_74_84();
    }
    conditional_code = equal(uniforms.f[93].xx, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_25_26();
    }
    reg_tmp9 = uniforms.f[74] + reg_tmp9;
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_16_21() {
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    if (uniforms.b[7]) {
        sub_18_19();
    }
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_18_19() {
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_25_26() {
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_28_74() {
    reg_tmp1 = uniforms.f[20];
    reg_tmp2 = uniforms.f[21];
    reg_tmp3 = uniforms.f[93].xxxx;
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop31 = 0u; loop31 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop31) {
        sub_32_73();
    }
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_32_73() {
    address_registers.x = (ivec2(reg_tmp3.xy)).x;
    reg_tmp4.x = (uniforms.f[69 + address_registers.x].wwww).x;
    reg_tmp4.y = (uniforms.f[71 + address_registers.x].wwww).y;
    conditional_code = equal(uniforms.f[93].xy, reg_tmp4.xy);
    if (conditional_code.x) {
        sub_37_39();
    } else {
        sub_39_63();
    }
    conditional_code.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
    conditional_code.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
    if (conditional_code.y) {
        sub_65_72();
    }
    reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
    return false;
}
bool sub_37_39() {
    reg_tmp6.x = dot_3(uniforms.f[69 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    return false;
}
bool sub_39_63() {
    reg_tmp4 = uniforms.f[69 + address_registers.x] + -reg_tmp15;
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    if (conditional_code.y) {
        sub_42_47();
    }
    reg_tmp5 = uniforms.f[70 + address_registers.x];
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.ww);
    reg_tmp4.w = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp4.w = rsq_s(reg_tmp4.w);
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp4.wwww);
    if (conditional_code.x) {
        sub_53_62();
    }
    reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp4.xyz);
    return false;
}
bool sub_42_47() {
    reg_tmp5.x = (uniforms.f[93].yyyy).x;
    reg_tmp5.z = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp5.y = (mul_s(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
    reg_tmp6.y = dot_3(uniforms.f[71 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.y = rcp_s(reg_tmp6.y);
    return false;
}
bool sub_53_62() {
    reg_tmp5.x = dot_3(uniforms.f[70 + address_registers.x].xyz, -reg_tmp4.xyz);
    reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx, uniforms.f[72 + address_registers.x].yyyy))).y;
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.xy);
    if (conditional_code.y) {
        sub_57_58();
    } else {
        sub_58_61();
    }
    reg_tmp6.y = (mul_s(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
    return false;
}
bool sub_57_58() {
    reg_tmp5.x = (uniforms.f[93].xxxx).x;
    return false;
}
bool sub_58_61() {
    reg_tmp5.y = log2(reg_tmp5.x);
    reg_tmp5.y = (mul_s(uniforms.f[72 + address_registers.x].xxxx, reg_tmp5.yyyy)).y;
    reg_tmp5.x = exp2(reg_tmp5.y);
    return false;
}
bool sub_65_72() {
    reg_tmp6.x = (max(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
    reg_tmp9.xyz = (fma_s(reg_tmp1.xyzz, uniforms.f[67 + address_registers.x].xyzz, reg_tmp9.xyzz)).xyz;
    reg_tmp4 = mul_s(uniforms.f[68 + address_registers.x], reg_tmp2);
    reg_tmp5.xyz = (mul_s(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
    reg_tmp5.xyz = (mul_s(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
    reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
    reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
    return false;
}
bool sub_74_84() {
    reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
    reg_tmp2 = uniforms.f[24].wwww;
    reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
    reg_tmp3 = uniforms.f[22];
    reg_tmp2 = uniforms.f[23] + -reg_tmp3;
    reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
    if (uniforms.b[6]) {
        sub_81_82();
    }
    reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_81_82() {
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
    return false;
}
bool sub_84_111() {
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    if (uniforms.b[9]) {
        sub_86_91();
    } else {
        sub_91_110();
    }
    return false;
}
bool sub_86_91() {
    {
        sub_111_119();
    }
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_91_110() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    if (all(not(conditional_code))) {
        sub_94_100();
    } else {
        sub_100_109();
    }
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_94_100() {
    reg_tmp6 = reg_tmp10;
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_100_109() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_101_105();
    } else {
        sub_105_108();
    }
    return false;
}
bool sub_101_105() {
    {
        sub_119_126();
    }
    reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
    reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
    reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
    return false;
}
bool sub_105_108() {
    {
        sub_126_130();
    }
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_111_119() {
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_113_114();
    } else {
        sub_114_118();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_113_114() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_114_118() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_115_116();
    } else {
        sub_116_117();
    }
    return false;
}
bool sub_115_116() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_116_117() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_119_126() {
    reg_tmp2 = -reg_tmp15;
    reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
    reg_tmp2.w = rsq_s(reg_tmp2.w);
    reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
    reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_126_130() {
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_130_154() {
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    if (uniforms.b[10]) {
        sub_132_136();
    } else {
        sub_136_153();
    }
    return false;
}
bool sub_132_136() {
    {
        sub_111_119();
    }
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_136_153() {
    if (uniforms.b[13]) {
        sub_137_151();
    } else {
        sub_151_152();
    }
    return false;
}
bool sub_137_151() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    if (all(not(conditional_code))) {
        sub_140_147();
    } else {
        sub_147_150();
    }
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_140_147() {
    reg_tmp6 = reg_tmp10;
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
    reg_tmp6.w = rcp_s(reg_tmp4.z);
    reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_147_150() {
    {
        sub_126_130();
    }
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    return false;
}
bool sub_151_152() {
    vs_out_attr5 = uniforms.f[93].xxxx;
    return false;
}
bool sub_154_170() {
    reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
    if (uniforms.b[11]) {
        sub_156_160();
    } else {
        sub_160_169();
    }
    return false;
}
bool sub_156_160() {
    {
        sub_111_119();
    }
    reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_160_169() {
    if (uniforms.b[14]) {
        sub_161_167();
    } else {
        sub_167_168();
    }
    return false;
}
bool sub_161_167() {
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp5.zw = (reg_tmp6.zwww).zw;
    {
        sub_126_130();
    }
    reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_167_168() {
    vs_out_attr6 = uniforms.f[93].xxxx;
    return false;
}
// reference: EAD34DEAF70733CA, 1993E2D62BB91686
// shader: 8B30, 1EF00ECA313C8C58

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - texcolor0.rrr) - (vec3(1.0) - texcolor1.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((1.0 - texcolor0.r) - (1.0 - texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 4.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(1.0)) * (vec3(1.0) - texcolor0.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (vec3(1.0) - last_tex_env_out.rgb);
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb) + (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((1.0 - last_tex_env_out.r) - (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B1A5DBDE4D845B60, 1EF00ECA313C8C58
// program: 1993E2D62BB91686, B80FA1B7F97410CD, 1EF00ECA313C8C58
// reference: A487B2AAAF4BF4FF, 1993E2D62BB91686
// shader: 8B30, B48E33664A2FB369

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - texcolor0.rrr) - (vec3(1.0) - texcolor1.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((1.0 - texcolor0.r) - (1.0 - texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 4.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(1.0)) * (vec3(1.0) - texcolor0.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - last_tex_env_out.r) + (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1EC8268EF7368957, B48E33664A2FB369
// program: 1993E2D62BB91686, B80FA1B7F97410CD, B48E33664A2FB369
// reference: 0E1FACB6A18BEF4D, 9583E5EAA2BC5470
// shader: 8B30, AE7E702E7EF58F31

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (primary_fragment_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5978E236DA580ADF, AE7E702E7EF58F31
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, AE7E702E7EF58F31
// shader: 8B31, 08BCAB0557C02D18

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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
bool sub_34_254();
bool sub_40_176();
bool sub_47_86();
bool sub_58_62();
bool sub_86_165();
bool sub_87_129();
bool sub_98_102();
bool sub_129_164();
bool sub_140_144();
bool sub_174_175();
bool sub_176_253();
bool sub_179_189();
bool sub_189_194();
bool sub_205_217();
bool sub_217_242();
bool sub_218_236();
bool sub_236_241();
bool sub_251_252();
bool sub_254_271();
bool sub_271_347();
bool sub_310_330();
bool sub_311_316();
bool sub_316_329();
bool sub_320_323();
bool sub_323_328();
bool sub_330_343();
bool sub_331_336();
bool sub_336_342();
bool sub_347_369();
bool sub_353_358();
bool sub_355_356();
bool sub_361_362();
bool sub_369_415();
bool sub_373_414();
bool sub_378_380();
bool sub_380_404();
bool sub_383_388();
bool sub_394_403();
bool sub_398_399();
bool sub_399_402();
bool sub_406_413();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    {
        sub_34_254();
    }
    {
        sub_347_369();
    }
    vs_out_attr4 = uniforms.f[93].xxxx;
    vs_out_attr5 = uniforms.f[93].xxxx;
    vs_out_attr6 = uniforms.f[93].xxxx;
    return true;
}
bool sub_7_12() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp5.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_254() {
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    if (uniforms.b[1]) {
        sub_40_176();
    } else {
        sub_176_253();
    }
    return false;
}
bool sub_40_176() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    reg_tmp7 = uniforms.f[93].xxxx;
    reg_tmp12 = uniforms.f[93].xxxx;
    reg_tmp11 = uniforms.f[93].xxxx;
    reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_86();
    } else {
        sub_86_165();
    }
    vs_out_attr2 = -reg_tmp15;
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_174_175();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_47_86() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_12_21();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_12_21();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_12_21();
    }
    if (uniforms.b[8]) {
        sub_58_62();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp6.y = (-uniforms.f[85].wwww + reg_tmp10.yyyy).y;
    reg_tmp9.xyz = (uniforms.f[85].xyzz).xyz;
    reg_tmp7.x = dot_s(reg_tmp9.xyzz, reg_tmp9.xyzz);
    reg_tmp7.x = rsq_s(reg_tmp7.x);
    reg_tmp7.xyz = (mul_s(reg_tmp9.xyzz, reg_tmp7.xxxx)).xyz;
    reg_tmp8.y = rcp_s(-reg_tmp7.y);
    reg_tmp8.y = (mul_s(reg_tmp6.yyyy, reg_tmp8.yyyy)).y;
    reg_tmp7.xyz = (mul_s(reg_tmp7.xyzz, reg_tmp8.yyyy)).xyz;
    reg_tmp10.xyz = (reg_tmp10.xyzz + reg_tmp7.xyzz).xyz;
    reg_tmp10.y = (uniforms.f[94].zzzz + reg_tmp10.yyyy).y;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_254_271();
    }
    return false;
}
bool sub_58_62() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_12_21();
    }
    return false;
}
bool sub_86_165() {
    if (all(conditional_code)) {
        sub_87_129();
    } else {
        sub_129_164();
    }
    return false;
}
bool sub_87_129() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_21_34();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_21_34();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_21_34();
    }
    if (uniforms.b[8]) {
        sub_98_102();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp6.y = (-uniforms.f[85].wwww + reg_tmp10.yyyy).y;
    reg_tmp9.xyz = (uniforms.f[85].xyzz).xyz;
    reg_tmp7.x = dot_s(reg_tmp9.xyzz, reg_tmp9.xyzz);
    reg_tmp7.x = rsq_s(reg_tmp7.x);
    reg_tmp7.xyz = (mul_s(reg_tmp9.xyzz, reg_tmp7.xxxx)).xyz;
    reg_tmp8.y = rcp_s(-reg_tmp7.y);
    reg_tmp8.y = (mul_s(reg_tmp6.yyyy, reg_tmp8.yyyy)).y;
    reg_tmp7.xyz = (mul_s(reg_tmp7.xyzz, reg_tmp8.yyyy)).xyz;
    reg_tmp10.xyz = (reg_tmp10.xyzz + reg_tmp7.xyzz).xyz;
    reg_tmp10.y = (uniforms.f[94].zzzz + reg_tmp10.yyyy).y;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    {
        sub_271_347();
    }
    return false;
}
bool sub_98_102() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_21_34();
    }
    return false;
}
bool sub_129_164() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_7_12();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_7_12();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_7_12();
    }
    if (uniforms.b[8]) {
        sub_140_144();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp6.y = (-uniforms.f[85].wwww + reg_tmp10.yyyy).y;
    reg_tmp9.xyz = (uniforms.f[85].xyzz).xyz;
    reg_tmp7.x = dot_s(reg_tmp9.xyzz, reg_tmp9.xyzz);
    reg_tmp7.x = rsq_s(reg_tmp7.x);
    reg_tmp7.xyz = (mul_s(reg_tmp9.xyzz, reg_tmp7.xxxx)).xyz;
    reg_tmp8.y = rcp_s(-reg_tmp7.y);
    reg_tmp8.y = (mul_s(reg_tmp6.yyyy, reg_tmp8.yyyy)).y;
    reg_tmp7.xyz = (mul_s(reg_tmp7.xyzz, reg_tmp8.yyyy)).xyz;
    reg_tmp10.xyz = (reg_tmp10.xyzz + reg_tmp7.xyzz).xyz;
    reg_tmp10.y = (uniforms.f[94].zzzz + reg_tmp10.yyyy).y;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_140_144() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_7_12();
    }
    return false;
}
bool sub_174_175() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_176_253() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    if (uniforms.b[2]) {
        sub_179_189();
    } else {
        sub_189_194();
    }
    reg_tmp6.y = (-uniforms.f[85].wwww + reg_tmp10.yyyy).y;
    reg_tmp9.xyz = (uniforms.f[85].xyzz).xyz;
    reg_tmp7.x = dot_s(reg_tmp9.xyzz, reg_tmp9.xyzz);
    reg_tmp7.x = rsq_s(reg_tmp7.x);
    reg_tmp7.xyz = (mul_s(reg_tmp9.xyzz, reg_tmp7.xxxx)).xyz;
    reg_tmp8.y = rcp_s(-reg_tmp7.y);
    reg_tmp8.y = (mul_s(reg_tmp6.yyyy, reg_tmp8.yyyy)).y;
    reg_tmp7.xyz = (mul_s(reg_tmp7.xyzz, reg_tmp8.yyyy)).xyz;
    reg_tmp10.xyz = (reg_tmp10.xyzz + reg_tmp7.xyzz).xyz;
    reg_tmp10.y = (uniforms.f[94].zzzz + reg_tmp10.yyyy).y;
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_205_217();
    } else {
        sub_217_242();
    }
    vs_out_attr2 = -reg_tmp15;
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_251_252();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_179_189() {
    reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp7.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp7.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp7.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_189_194() {
    address_registers.x = (ivec2(uniforms.f[93].xx)).x;
    reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
    reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
    reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_205_217() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_254_271();
    }
    return false;
}
bool sub_217_242() {
    if (all(conditional_code)) {
        sub_218_236();
    } else {
        sub_236_241();
    }
    return false;
}
bool sub_218_236() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp11.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    {
        sub_271_347();
    }
    return false;
}
bool sub_236_241() {
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_251_252() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_254_271() {
    uint jmp_to = 254u;
    while (true) {
        switch (jmp_to) {
        case 254u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 270u; break; }
            }
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
            conditional_code = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
            if (conditional_code.x) {
                { jmp_to = 270u; break; }
            }
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 270u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_271_347() {
    uint jmp_to = 271u;
    while (true) {
        switch (jmp_to) {
        case 271u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 346u; break; }
            }
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
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
            conditional_code = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            if (!conditional_code.x) {
                { jmp_to = 308u; break; }
            }
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            reg_tmp7.w = (reg_tmp6).w;
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            if (uniforms.b[0]) {
                { jmp_to = 346u; break; }
            }
        }
        case 308u: {
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            if (conditional_code.x) {
                sub_310_330();
            } else {
                sub_330_343();
            }
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 346u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_310_330() {
    if (conditional_code.y) {
        sub_311_316();
    } else {
        sub_316_329();
    }
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_311_316() {
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_316_329() {
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    if (conditional_code.x) {
        sub_320_323();
    } else {
        sub_323_328();
    }
    return false;
}
bool sub_320_323() {
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_323_328() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_330_343() {
    if (conditional_code.y) {
        sub_331_336();
    } else {
        sub_336_342();
    }
    return false;
}
bool sub_331_336() {
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_336_342() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_347_369() {
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    if (conditional_code.y) {
        sub_353_358();
    }
    if (uniforms.b[12]) {
        sub_369_415();
    }
    conditional_code = equal(uniforms.f[93].xx, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_361_362();
    }
    reg_tmp0.xyz = (uniforms.f[24].xyzz + -reg_tmp10.xyzz).xyz;
    reg_tmp0.w = rcp_s(uniforms.f[24].w);
    reg_tmp1.w = dot_3(reg_tmp0.xyz, reg_tmp0.xyz);
    reg_tmp1.w = (mul_s(reg_tmp0.wwww, reg_tmp1.wwww)).w;
    reg_tmp1.w = (min(uniforms.f[93].yyyy, reg_tmp1.wwww)).w;
    reg_tmp9.w = (uniforms.f[93].yyyy + -reg_tmp1.wwww).w;
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_353_358() {
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    if (uniforms.b[7]) {
        sub_355_356();
    }
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_355_356() {
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_361_362() {
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_369_415() {
    reg_tmp1 = uniforms.f[20];
    reg_tmp2 = uniforms.f[21];
    reg_tmp3 = uniforms.f[93].xxxx;
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop372 = 0u; loop372 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop372) {
        sub_373_414();
    }
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_373_414() {
    address_registers.x = (ivec2(reg_tmp3.xy)).x;
    reg_tmp4.x = (uniforms.f[81 + address_registers.x].wwww).x;
    reg_tmp4.y = (uniforms.f[83 + address_registers.x].wwww).y;
    conditional_code = equal(uniforms.f[93].xy, reg_tmp4.xy);
    if (conditional_code.x) {
        sub_378_380();
    } else {
        sub_380_404();
    }
    conditional_code.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
    conditional_code.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
    if (conditional_code.y) {
        sub_406_413();
    }
    reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
    return false;
}
bool sub_378_380() {
    reg_tmp6.x = dot_3(uniforms.f[81 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    return false;
}
bool sub_380_404() {
    reg_tmp4 = uniforms.f[81 + address_registers.x] + -reg_tmp15;
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    if (conditional_code.y) {
        sub_383_388();
    }
    reg_tmp5 = uniforms.f[82 + address_registers.x];
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.ww);
    reg_tmp4.w = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp4.w = rsq_s(reg_tmp4.w);
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp4.wwww);
    if (conditional_code.x) {
        sub_394_403();
    }
    reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp4.xyz);
    return false;
}
bool sub_383_388() {
    reg_tmp5.x = (uniforms.f[93].yyyy).x;
    reg_tmp5.z = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp5.y = (mul_s(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
    reg_tmp6.y = dot_3(uniforms.f[83 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.y = rcp_s(reg_tmp6.y);
    return false;
}
bool sub_394_403() {
    reg_tmp5.x = dot_3(uniforms.f[82 + address_registers.x].xyz, -reg_tmp4.xyz);
    reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx, uniforms.f[84 + address_registers.x].yyyy))).y;
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.xy);
    if (conditional_code.y) {
        sub_398_399();
    } else {
        sub_399_402();
    }
    reg_tmp6.y = (mul_s(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
    return false;
}
bool sub_398_399() {
    reg_tmp5.x = (uniforms.f[93].xxxx).x;
    return false;
}
bool sub_399_402() {
    reg_tmp5.y = log2(reg_tmp5.x);
    reg_tmp5.y = (mul_s(uniforms.f[84 + address_registers.x].xxxx, reg_tmp5.yyyy)).y;
    reg_tmp5.x = exp2(reg_tmp5.y);
    return false;
}
bool sub_406_413() {
    reg_tmp6.x = (max(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
    reg_tmp9.xyz = (fma_s(reg_tmp1.xyzz, uniforms.f[79 + address_registers.x].xyzz, reg_tmp9.xyzz)).xyz;
    reg_tmp4 = mul_s(uniforms.f[80 + address_registers.x], reg_tmp2);
    reg_tmp5.xyz = (mul_s(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
    reg_tmp5.xyz = (mul_s(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
    reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
    reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
    return false;
}
// reference: 3E3C36C7FBFE9E66, 08BCAB0557C02D18
// shader: 8B30, 355EF9468697FD85

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - rounded_primary_color.aaa) + (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (vec3(1.0) - const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8020625A39D3AA43, 355EF9468697FD85
// program: 08BCAB0557C02D18, B80FA1B7F97410CD, 355EF9468697FD85
// shader: 8B30, BD509CDCE8AC9EAA

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - rounded_primary_color.aaa) + (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (vec3(1.0) - const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8020625AE8D9B5BF, BD509CDCE8AC9EAA
// program: 0000000000000000, 0000000000000000, BD509CDCE8AC9EAA
// reference: B70A0F52E3A293C6, 60F5295A12916EF0
// shader: 8B30, 6EA4E7C7936DB486

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - rounded_primary_color.aaa) + (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (vec3(1.0) - const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7FE8CEB7D506E1D, 6EA4E7C7936DB486
// program: 60F5295A12916EF0, 42937135801BAA7E, 6EA4E7C7936DB486
// reference: F95EF012BBEE54F3, 60F5295A12916EF0
// shader: 8B30, C2C82B7F0731E872

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - rounded_primary_color.aaa) + (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (vec3(1.0) - const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7A438EF5B82A7E0A, C2C82B7F0731E872
// program: 0000000000000000, 0000000000000000, C2C82B7F0731E872
// reference: CE0AC5F582BBD02C, 9583E5EAA2BC5470
// reference: 805E3AB5DAF71719, 9583E5EAA2BC5470
// reference: DAB3C4835EB3197D, 3B47F23567C9D426
// reference: 487D8D2DCC7880E5, 4ACE9F564CD31D6B
// reference: C63F3C135E1DA9CE, 4ACE9F564CD31D6B
// reference: 943A9788244FE826, 46156D30B20FEAE2
// reference: C40776ED2B3CAACA, ABE439554EDF4B6D
// reference: CC6E6D61A78570EC, 4E545413B902FBCE
// reference: 984E47EBF9C72878, 9583E5EAA2BC5470
// reference: 943A9788C8F80325, FBB5B6DEC0AF5970
// reference: 370AD5AC3841945A, DD1B5AA99EDA2A36
// reference: 370AD5ACD778071B, DD1B5AA99EDA2A36
// reference: 984E47EB16FEBB39, 9583E5EAA2BC5470
// reference: 893250925EB3197D, 05EE70C9287E57E8
// reference: 3F4D457F0F7A9D6C, 412420884B7D7C0F
// reference: 89325092B204F27E, DCD1284C69838DDF
// reference: 3A31C41C244FE826, 46156D30B20FEAE2
// reference: FFF7BA5300F9FFD5, D41733ECAC30BB84
// reference: D5E7AAC83318B62E, 19B8576E9F00D1AF
// reference: 86663ED93318B62E, EBBDAECFBE86663B
// reference: 3A925354BBDD8155, 05378086BF2F4B98
// shader: 8B30, 98A6D1D7B328F449

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (1.0 - texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 30195BBB1519A68B, 98A6D1D7B328F449
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 98A6D1D7B328F449
// reference: 32D659F7F70733CA, 1993E2D62BB91686
// reference: 7C82A6B7AF4BF4FF, 1993E2D62BB91686
// reference: D61AB8ABA18BEF4D, 9583E5EAA2BC5470
// reference: A0327416A63DB4E7, 3119C0AC5ED36E98
// reference: C94B7F73D48D6D2B, 3119C0AC5ED36E98
// reference: 87C274DABDF4664E, 3119C0AC5ED36E98
// reference: 87C274DAD48D6D2B, 3119C0AC5ED36E98
// reference: A0327416D48D6D2B, 3119C0AC5ED36E98
// shader: 8B30, 7461932886609B06

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - rounded_primary_color.aaa) + (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (vec3(1.0) - const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7FE8CEB33A8E707, 7461932886609B06
// program: 60F5295A12916EF0, 42937135801BAA7E, 7461932886609B06
// shader: 8B30, A40D69702D9E7DAA

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - rounded_primary_color.aaa) + (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (vec3(1.0) - const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 38666814FCAF78A7, A40D69702D9E7DAA
// program: 60F5295A12916EF0, 42937135801BAA7E, A40D69702D9E7DAA
// reference: 63CE8E81BF82DD7E, B37B323413853CE0
// shader: 8B30, F15EE23489D482E2

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5989E34F598C7306, F15EE23489D482E2
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, F15EE23489D482E2
// reference: BE081563CF44BF82, 3119C0AC5ED36E98
// reference: 2F1AA8F21B71AB3B, DD1B5AA99EDA2A36
// reference: 805E3AB535CE8458, 9583E5EAA2BC5470
// reference: 4FC2F057E7D5718A, C4555434E80CC41C
// reference: E379FD983F863A8F, 46156D30B20FEAE2
// reference: 99F815AFD48D6D2B, 3119C0AC5ED36E98
// reference: BE081563A63DB4E7, 3119C0AC5ED36E98
// reference: CC6E6D6148BCE3AD, 4E545413B902FBCE
// reference: 823A9221FFC9B7D9, 4E545413B902FBCE
// reference: 337DE45DCD3204B2, B37B323413853CE0
// shader: 8B31, 2B6536D112FF87FD

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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
bool sub_34_73();
bool sub_73_90();
bool sub_166_183();
bool sub_172_177();
bool sub_174_175();
bool sub_181_182();
bool sub_183_229();
bool sub_187_228();
bool sub_192_194();
bool sub_194_218();
bool sub_197_202();
bool sub_208_217();
bool sub_212_213();
bool sub_213_216();
bool sub_220_227();
bool sub_229_239();
bool sub_236_237();
bool sub_239_266();
bool sub_241_246();
bool sub_246_265();
bool sub_249_255();
bool sub_255_264();
bool sub_256_260();
bool sub_260_263();
bool sub_266_274();
bool sub_268_269();
bool sub_269_273();
bool sub_270_271();
bool sub_271_272();
bool sub_274_281();
bool sub_281_285();
bool sub_285_309();
bool sub_287_291();
bool sub_291_308();
bool sub_292_306();
bool sub_295_302();
bool sub_302_305();
bool sub_306_307();
bool sub_309_325();
bool sub_311_315();
bool sub_315_324();
bool sub_316_322();
bool sub_322_323();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    {
        sub_34_73();
    }
    {
        sub_166_183();
    }
    {
        sub_239_266();
    }
    {
        sub_285_309();
    }
    {
        sub_309_325();
    }
    return true;
}
bool sub_34_73() {
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp0 = uniforms.f[7];
    reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
    reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
    reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp12.x = dot_3(uniforms.f[25].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27].xyz, reg_tmp14.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_73_90();
    }
    vs_out_attr2 = -reg_tmp15;
    reg_tmp0.x = (uniforms.f[93].yyyy).x;
    address_registers.x = (ivec2(vs_in_reg3.xx)).x;
    reg_tmp0.x = (mul_s(uniforms.f[28 + address_registers.x].xxxx, reg_tmp0.xxxx)).x;
    address_registers.x = (ivec2(vs_in_reg3.yy)).x;
    reg_tmp0.x = (mul_s(uniforms.f[28 + address_registers.x].yyyy, reg_tmp0.xxxx)).x;
    address_registers.x = (ivec2(vs_in_reg3.zz)).x;
    reg_tmp0.x = (mul_s(uniforms.f[28 + address_registers.x].zzzz, reg_tmp0.xxxx)).x;
    address_registers.x = (ivec2(vs_in_reg3.ww)).x;
    reg_tmp0.x = (mul_s(uniforms.f[28 + address_registers.x].wwww, reg_tmp0.xxxx)).x;
    reg_tmp15.xyz = (mul_s(reg_tmp15.xyzz, reg_tmp0.xxxx)).xyz;
    vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
    vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
    vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
    vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_73_90() {
    uint jmp_to = 73u;
    while (true) {
        switch (jmp_to) {
        case 73u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 89u; break; }
            }
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
            conditional_code = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
            if (conditional_code.x) {
                { jmp_to = 89u; break; }
            }
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 89u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_166_183() {
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    if (conditional_code.y) {
        sub_172_177();
    }
    if (uniforms.b[12]) {
        sub_183_229();
    }
    if (uniforms.b[5]) {
        sub_229_239();
    }
    conditional_code = equal(uniforms.f[93].xx, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_181_182();
    }
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_172_177() {
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    if (uniforms.b[7]) {
        sub_174_175();
    }
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_174_175() {
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_181_182() {
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_183_229() {
    reg_tmp1 = uniforms.f[20];
    reg_tmp2 = uniforms.f[21];
    reg_tmp3 = uniforms.f[93].xxxx;
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop186 = 0u; loop186 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop186) {
        sub_187_228();
    }
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_187_228() {
    address_registers.x = (ivec2(reg_tmp3.xy)).x;
    reg_tmp4.x = (uniforms.f[49 + address_registers.x].wwww).x;
    reg_tmp4.y = (uniforms.f[51 + address_registers.x].wwww).y;
    conditional_code = equal(uniforms.f[93].xy, reg_tmp4.xy);
    if (conditional_code.x) {
        sub_192_194();
    } else {
        sub_194_218();
    }
    conditional_code.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
    conditional_code.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
    if (conditional_code.y) {
        sub_220_227();
    }
    reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
    return false;
}
bool sub_192_194() {
    reg_tmp6.x = dot_3(uniforms.f[49 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    return false;
}
bool sub_194_218() {
    reg_tmp4 = uniforms.f[49 + address_registers.x] + -reg_tmp15;
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    if (conditional_code.y) {
        sub_197_202();
    }
    reg_tmp5 = uniforms.f[50 + address_registers.x];
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.ww);
    reg_tmp4.w = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp4.w = rsq_s(reg_tmp4.w);
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp4.wwww);
    if (conditional_code.x) {
        sub_208_217();
    }
    reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp4.xyz);
    return false;
}
bool sub_197_202() {
    reg_tmp5.x = (uniforms.f[93].yyyy).x;
    reg_tmp5.z = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp5.y = (mul_s(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
    reg_tmp6.y = dot_3(uniforms.f[51 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.y = rcp_s(reg_tmp6.y);
    return false;
}
bool sub_208_217() {
    reg_tmp5.x = dot_3(uniforms.f[50 + address_registers.x].xyz, -reg_tmp4.xyz);
    reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx, uniforms.f[52 + address_registers.x].yyyy))).y;
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.xy);
    if (conditional_code.y) {
        sub_212_213();
    } else {
        sub_213_216();
    }
    reg_tmp6.y = (mul_s(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
    return false;
}
bool sub_212_213() {
    reg_tmp5.x = (uniforms.f[93].xxxx).x;
    return false;
}
bool sub_213_216() {
    reg_tmp5.y = log2(reg_tmp5.x);
    reg_tmp5.y = (mul_s(uniforms.f[52 + address_registers.x].xxxx, reg_tmp5.yyyy)).y;
    reg_tmp5.x = exp2(reg_tmp5.y);
    return false;
}
bool sub_220_227() {
    reg_tmp6.x = (max(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
    reg_tmp9.xyz = (fma_s(reg_tmp1.xyzz, uniforms.f[47 + address_registers.x].xyzz, reg_tmp9.xyzz)).xyz;
    reg_tmp4 = mul_s(uniforms.f[48 + address_registers.x], reg_tmp2);
    reg_tmp5.xyz = (mul_s(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
    reg_tmp5.xyz = (mul_s(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
    reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
    reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
    return false;
}
bool sub_229_239() {
    reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
    reg_tmp2 = uniforms.f[24].wwww;
    reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
    reg_tmp3 = uniforms.f[22];
    reg_tmp2 = uniforms.f[23] + -reg_tmp3;
    reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
    if (uniforms.b[6]) {
        sub_236_237();
    }
    reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_236_237() {
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
    return false;
}
bool sub_239_266() {
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    if (uniforms.b[9]) {
        sub_241_246();
    } else {
        sub_246_265();
    }
    return false;
}
bool sub_241_246() {
    {
        sub_266_274();
    }
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_246_265() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    if (all(not(conditional_code))) {
        sub_249_255();
    } else {
        sub_255_264();
    }
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_249_255() {
    reg_tmp6 = reg_tmp10;
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_255_264() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_256_260();
    } else {
        sub_260_263();
    }
    return false;
}
bool sub_256_260() {
    {
        sub_274_281();
    }
    reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
    reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
    reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
    return false;
}
bool sub_260_263() {
    {
        sub_281_285();
    }
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_266_274() {
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_268_269();
    } else {
        sub_269_273();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_268_269() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_269_273() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_270_271();
    } else {
        sub_271_272();
    }
    return false;
}
bool sub_270_271() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_271_272() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_274_281() {
    reg_tmp2 = -reg_tmp15;
    reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
    reg_tmp2.w = rsq_s(reg_tmp2.w);
    reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
    reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_281_285() {
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_285_309() {
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    if (uniforms.b[10]) {
        sub_287_291();
    } else {
        sub_291_308();
    }
    return false;
}
bool sub_287_291() {
    {
        sub_266_274();
    }
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_291_308() {
    if (uniforms.b[13]) {
        sub_292_306();
    } else {
        sub_306_307();
    }
    return false;
}
bool sub_292_306() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    if (all(not(conditional_code))) {
        sub_295_302();
    } else {
        sub_302_305();
    }
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_295_302() {
    reg_tmp6 = reg_tmp10;
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
    reg_tmp6.w = rcp_s(reg_tmp4.z);
    reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_302_305() {
    {
        sub_281_285();
    }
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    return false;
}
bool sub_306_307() {
    vs_out_attr5 = uniforms.f[93].xxxx;
    return false;
}
bool sub_309_325() {
    reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
    if (uniforms.b[11]) {
        sub_311_315();
    } else {
        sub_315_324();
    }
    return false;
}
bool sub_311_315() {
    {
        sub_266_274();
    }
    reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_315_324() {
    if (uniforms.b[14]) {
        sub_316_322();
    } else {
        sub_322_323();
    }
    return false;
}
bool sub_316_322() {
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp5.zw = (reg_tmp6.zwww).zw;
    {
        sub_281_285();
    }
    reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_322_323() {
    vs_out_attr6 = uniforms.f[93].xxxx;
    return false;
}
// reference: 7B50D9DEBAE8966E, 2B6536D112FF87FD
// shader: 8B30, 63EC5B1DEC79BC4F

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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (secondary_fragment_color.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 481A2F7F75B058A1, 63EC5B1DEC79BC4F
// program: 2B6536D112FF87FD, B80FA1B7F97410CD, 63EC5B1DEC79BC4F
// reference: 37A51738A78570EC, 4E545413B902FBCE
// reference: 63853DB2F9C72878, 9583E5EAA2BC5470
// reference: CCC1AFF53841945A, DD1B5AA99EDA2A36
// reference: CCC1AFF5D778071B, DD1B5AA99EDA2A36
// reference: 63853DB216FEBB39, 9583E5EAA2BC5470
// shader: 8B30, A1A96E2C7CB4C6DF

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 70284790F2EDAD59, A1A96E2C7CB4C6DF
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, A1A96E2C7CB4C6DF
// shader: 8B30, 9DC2199FECA84866

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor1.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7028479027403241, 9DC2199FECA84866
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 9DC2199FECA84866
// shader: 8B30, D0B5CBB5D9352AD3

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(view))));
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - secondary_fragment_color.aaa) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor1.a) * (1.0 - secondary_fragment_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor2.a) * (secondary_fragment_color.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.aaa) * (texcolor2.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3C4DE45023948EEF, D0B5CBB5D9352AD3
// program: DD1B5AA99EDA2A36, B80FA1B7F97410CD, D0B5CBB5D9352AD3
// shader: 8B30, 3FCCC0236A3B624E

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.aaa) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.aaa);
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((combiner_buffer.rgb) * (vec3(1.0) - last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A2BAD562BC24034E, 3FCCC0236A3B624E
// program: DD1B5AA99EDA2A36, B80FA1B7F97410CD, 3FCCC0236A3B624E
// reference: C91D23AEF70733CA, 1993E2D62BB91686
// reference: 8749DCEEAF4BF4FF, 1993E2D62BB91686
// reference: 2DD1C2F2A18BEF4D, 9583E5EAA2BC5470
// reference: 3F59456F0700816D, 63EC5B1DEC79BC4F
// reference: 70284790805D7495, A1A96E2C7CB4C6DF
// reference: 3C4DE4507327E433, D0B5CBB5D9352AD3
// reference: A2BAD562EC976992, 3FCCC0236A3B624E
// reference: 7028479055F0EB8D, 9DC2199FECA84866
// shader: 8B30, 01BDA6918F5CF5AC

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B3CACFE6C0B42BB2, 01BDA6918F5CF5AC
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 01BDA6918F5CF5AC
// reference: DAB3C483C0B42BB2, 01BDA6918F5CF5AC
// shader: 8B30, 46C2E683FA6639F1

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 473D51212C03C0B1, 46C2E683FA6639F1
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 46C2E683FA6639F1
// shader: 8B30, 1C81792A8869223A

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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor1.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((texcolor1.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4E5901733C16DE87, 1C81792A8869223A
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 1C81792A8869223A
// reference: D290B1ECA8839EBD, 47C493E4C6F402FD
// reference: 473D51215EB3197D, 46C2E683FA6639F1
// reference: 4E5901734EA6074B, 1C81792A8869223A
// reference: 8A00AE5FDB7DF91B, 01BDA6918F5CF5AC
// reference: DAB3C483A9CD20D7, 01BDA6918F5CF5AC
// reference: D7711E06D48D6D2B, 3119C0AC5ED36E98
// reference: 27200A163C16DE87, 1C81792A8869223A
// shader: 8B30, C18D3D2A00004367

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
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E99905352C03C0B1, C18D3D2A00004367
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, C18D3D2A00004367
// shader: 8B30, FF1B1B7D4B7919D0

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0A08775E598C7306, FF1B1B7D4B7919D0
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, FF1B1B7D4B7919D0
// shader: 8B30, 74210693C7372778

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (secondary_fragment_color.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((primary_fragment_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FDA41AD4E0D9DD7E, 74210693C7372778
// program: 4E545413B902FBCE, D0BDF6740B5CE976, 74210693C7372778
// reference: E99905355EB3197D, C18D3D2A00004367
// reference: 337DE45DBF82DD7E, B37B323413853CE0
// reference: F0811ECACF44BF82, 3119C0AC5ED36E98
// reference: 9049968093BF16FD, 9583E5EAA2BC5470
// reference: 1CF819D893BF16FD, 9583E5EAA2BC5470
// reference: B908DBC8295E06BE, 2D9A3162586D2ED4
// shader: 8B30, 0BF1EB09F86FE320

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
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((rounded_primary_color.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9EF8DB042D2993F8, 0BF1EB09F86FE320
// program: 0000000000000000, 0000000000000000, 0BF1EB09F86FE320
// reference: 0E4DB6364806B7BA, 9583E5EAA2BC5470
// reference: EEC1A3144C9892AC, 60F5295A12916EF0
// reference: FC740CFA972133EB, 60F5295A12916EF0
// reference: 146B797C7C3CD1AB, 4E545413B902FBCE
// reference: 06DED692A78570EC, 4E545413B902FBCE
// reference: 404B53F6227E893F, 9583E5EAA2BC5470
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 1C81792A8869223A
// reference: 52FEFC18F9C72878, 9583E5EAA2BC5470
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, C18D3D2A00004367
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, FF1B1B7D4B7919D0
// program: 4E545413B902FBCE, D0BDF6740B5CE976, 74210693C7372778
// reference: A487B2AA74F255B8, 1993E2D62BB91686
// reference: B6321D44AF4BF4FF, 1993E2D62BB91686
// reference: 2C899929FBFE9E66, 08BCAB0557C02D18
// reference: 92EB955BDAF71719, 9583E5EAA2BC5470
// reference: F95EF0126057F5B4, 60F5295A12916EF0
// reference: EBEB5FFCBBEE54F3, 60F5295A12916EF0
// reference: CC6E6D617C3CD1AB, 4E545413B902FBCE
// reference: DEDBC28FA78570EC, 4E545413B902FBCE
// reference: 984E47EB227E893F, 9583E5EAA2BC5470
// reference: 8AFBE805F9C72878, 9583E5EAA2BC5470
// reference: 7C82A6B774F255B8, 1993E2D62BB91686
// reference: 6E370959AF4BF4FF, 1993E2D62BB91686
// reference: 63CE8E81EF31B7A2, B37B323413853CE0
// reference: 82FC396E93BF16FD, 9583E5EAA2BC5470
// reference: 0E4DB63693BF16FD, 9583E5EAA2BC5470
// program: 0000000000000000, 0000000000000000, 0BF1EB09F86FE320
// reference: FC740CFA4C9892AC, 60F5295A12916EF0
// reference: 06DED6927C3CD1AB, 4E545413B902FBCE
// reference: 52FEFC18227E893F, 9583E5EAA2BC5470
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 1C81792A8869223A
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, C18D3D2A00004367
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, FF1B1B7D4B7919D0
// program: 4E545413B902FBCE, D0BDF6740B5CE976, 74210693C7372778
// reference: B6321D4474F255B8, 1993E2D62BB91686
// reference: EBEB5FFC6057F5B4, 60F5295A12916EF0
// reference: DEDBC28F7C3CD1AB, 4E545413B902FBCE
// reference: 8AFBE805227E893F, 9583E5EAA2BC5470
// reference: 6E37095974F255B8, 1993E2D62BB91686
// reference: 92EB955B014EB65E, 9583E5EAA2BC5470
// shader: 8B30, 4E8B40EC05E52645

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;

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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec4(0.0).rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((vec4(0.0).rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (vec4(0.0).a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((rounded_primary_color.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AA232DC5F18A06F9, 4E8B40EC05E52645
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 4E8B40EC05E52645
// reference: 7627740757C26AD9, 1993E2D62BB91686
// reference: 6492DBE98C7BCB9E, 1993E2D62BB91686
// shader: 8B30, C507F6CE9C927C7B

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(vec3(dot((texcolor0.rgb) - vec3(0.5), (texcolor1.rgb) - vec3(0.5)) * 4.0), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) + (1.0 - texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp(vec3(dot((texcolor0.rgb) - vec3(0.5), (texcolor2.rgb) - vec3(0.5)) * 4.0), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp(min((texcolor1.a) + (1.0 - texcolor0.a), 1.0) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp(min((texcolor0.a) + (1.0 - texcolor2.a), 1.0) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp(min((texcolor2.a) + (1.0 - texcolor0.a), 1.0) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) - (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) - (primary_fragment_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5 * 1.0, alpha_output_5 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 51511626FE35BA0C, C507F6CE9C927C7B
// program: 1993E2D62BB91686, B80FA1B7F97410CD, C507F6CE9C927C7B
// shader: 8B30, 05ACA9E48DF2EA73

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - texcolor0.rrr) - (vec3(1.0) - texcolor1.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((1.0 - texcolor0.r) - (1.0 - texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 4.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(1.0)) * (vec3(1.0) - texcolor0.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - last_tex_env_out.r) + (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1EC8268EE4436578, 05ACA9E48DF2EA73
// program: 0000000000000000, 0000000000000000, 05ACA9E48DF2EA73
// shader: 8B30, 6C4F6E107EFBE36B

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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8BEDD5D8B95078CF, 6C4F6E107EFBE36B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 6C4F6E107EFBE36B
// reference: AA232DC5833ADF35, 4E8B40EC05E52645
// shader: 8B30, B50DFB3C2276697B

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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (primary_fragment_color.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 98156E2841DED555, B50DFB3C2276697B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, B50DFB3C2276697B
// shader: 8B30, 5D82DD64D93FB5CE

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 85A0C1FDA2F80DD4, 5D82DD64D93FB5CE
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 5D82DD64D93FB5CE
// reference: 8BEDD5D8CBE0A103, 6C4F6E107EFBE36B
// reference: 85A0C1FDD048D418, 5D82DD64D93FB5CE
// shader: 8B31, B6A06BC7A678BBFB

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_6_11();
bool sub_11_20();
bool sub_20_29();
bool sub_37_197();
bool sub_42_139();
bool sub_48_76();
bool sub_59_63();
bool sub_76_129();
bool sub_77_104();
bool sub_88_92();
bool sub_104_128();
bool sub_115_119();
bool sub_137_138();
bool sub_139_194();
bool sub_142_152();
bool sub_152_157();
bool sub_158_168();
bool sub_168_184();
bool sub_169_179();
bool sub_179_183();
bool sub_192_193();
bool sub_197_214();
bool sub_203_208();
bool sub_205_206();
bool sub_212_213();
bool sub_214_260();
bool sub_218_259();
bool sub_223_225();
bool sub_225_249();
bool sub_228_233();
bool sub_239_248();
bool sub_243_244();
bool sub_244_247();
bool sub_251_258();
bool sub_260_270();
bool sub_267_268();
bool sub_279_288();
bool sub_321_329();
bool sub_323_324();
bool sub_324_328();
bool sub_325_326();
bool sub_326_327();
bool sub_329_333();
bool sub_333_4096();
bool sub_339_366();
bool sub_341_346();
bool sub_346_365();
bool sub_349_355();
bool sub_355_364();
bool sub_356_360();
bool sub_360_363();
bool sub_366_373();

bool exec_shader() {
    sub_333_4096();
    return true;
}

bool sub_6_11() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_11_20() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_20_29() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_37_197() {
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    if (uniforms.b[1]) {
        sub_42_139();
    } else {
        sub_139_194();
    }
    vs_out_attr1 = uniforms.f[93].xxxx;
    vs_out_attr2 = uniforms.f[93].xxxx;
    return false;
}
bool sub_42_139() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    reg_tmp7 = uniforms.f[93].xxxx;
    reg_tmp12 = uniforms.f[93].xxxx;
    reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_48_76();
    } else {
        sub_76_129();
    }
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_137_138();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_48_76() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_11_20();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_11_20();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_11_20();
    }
    if (uniforms.b[8]) {
        sub_59_63();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_59_63() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_11_20();
    }
    return false;
}
bool sub_76_129() {
    if (all(conditional_code)) {
        sub_77_104();
    } else {
        sub_104_128();
    }
    return false;
}
bool sub_77_104() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_20_29();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_20_29();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_20_29();
    }
    if (uniforms.b[8]) {
        sub_88_92();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_88_92() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_20_29();
    }
    return false;
}
bool sub_104_128() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_6_11();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_6_11();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_6_11();
    }
    if (uniforms.b[8]) {
        sub_115_119();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_115_119() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_6_11();
    }
    return false;
}
bool sub_137_138() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_139_194() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    if (uniforms.b[2]) {
        sub_142_152();
    } else {
        sub_152_157();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_158_168();
    } else {
        sub_168_184();
    }
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_192_193();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_142_152() {
    reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp7.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp7.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp7.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_152_157() {
    address_registers.x = (ivec2(uniforms.f[93].xx)).x;
    reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
    reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
    reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_158_168() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_168_184() {
    if (all(conditional_code)) {
        sub_169_179();
    } else {
        sub_179_183();
    }
    return false;
}
bool sub_169_179() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_179_183() {
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_192_193() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_197_214() {
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    if (conditional_code.y) {
        sub_203_208();
    }
    if (uniforms.b[12]) {
        sub_214_260();
    }
    if (uniforms.b[5]) {
        sub_260_270();
    }
    conditional_code = equal(uniforms.f[93].xx, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_212_213();
    }
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_203_208() {
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    if (uniforms.b[7]) {
        sub_205_206();
    }
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_205_206() {
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_212_213() {
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_214_260() {
    reg_tmp1 = uniforms.f[20];
    reg_tmp2 = uniforms.f[21];
    reg_tmp3 = uniforms.f[93].xxxx;
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop217 = 0u; loop217 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop217) {
        sub_218_259();
    }
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_218_259() {
    address_registers.x = (ivec2(reg_tmp3.xy)).x;
    reg_tmp4.x = (uniforms.f[81 + address_registers.x].wwww).x;
    reg_tmp4.y = (uniforms.f[83 + address_registers.x].wwww).y;
    conditional_code = equal(uniforms.f[93].xy, reg_tmp4.xy);
    if (conditional_code.x) {
        sub_223_225();
    } else {
        sub_225_249();
    }
    conditional_code.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
    conditional_code.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
    if (conditional_code.y) {
        sub_251_258();
    }
    reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
    return false;
}
bool sub_223_225() {
    reg_tmp6.x = dot_3(uniforms.f[81 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    return false;
}
bool sub_225_249() {
    reg_tmp4 = uniforms.f[81 + address_registers.x] + -reg_tmp15;
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    if (conditional_code.y) {
        sub_228_233();
    }
    reg_tmp5 = uniforms.f[82 + address_registers.x];
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.ww);
    reg_tmp4.w = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp4.w = rsq_s(reg_tmp4.w);
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp4.wwww);
    if (conditional_code.x) {
        sub_239_248();
    }
    reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp4.xyz);
    return false;
}
bool sub_228_233() {
    reg_tmp5.x = (uniforms.f[93].yyyy).x;
    reg_tmp5.z = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp5.y = (mul_s(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
    reg_tmp6.y = dot_3(uniforms.f[83 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.y = rcp_s(reg_tmp6.y);
    return false;
}
bool sub_239_248() {
    reg_tmp5.x = dot_3(uniforms.f[82 + address_registers.x].xyz, -reg_tmp4.xyz);
    reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx, uniforms.f[84 + address_registers.x].yyyy))).y;
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.xy);
    if (conditional_code.y) {
        sub_243_244();
    } else {
        sub_244_247();
    }
    reg_tmp6.y = (mul_s(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
    return false;
}
bool sub_243_244() {
    reg_tmp5.x = (uniforms.f[93].xxxx).x;
    return false;
}
bool sub_244_247() {
    reg_tmp5.y = log2(reg_tmp5.x);
    reg_tmp5.y = (mul_s(uniforms.f[84 + address_registers.x].xxxx, reg_tmp5.yyyy)).y;
    reg_tmp5.x = exp2(reg_tmp5.y);
    return false;
}
bool sub_251_258() {
    reg_tmp6.x = (max(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
    reg_tmp9.xyz = (fma_s(reg_tmp1.xyzz, uniforms.f[79 + address_registers.x].xyzz, reg_tmp9.xyzz)).xyz;
    reg_tmp4 = mul_s(uniforms.f[80 + address_registers.x], reg_tmp2);
    reg_tmp5.xyz = (mul_s(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
    reg_tmp5.xyz = (mul_s(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
    reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
    reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
    return false;
}
bool sub_260_270() {
    reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
    reg_tmp2 = uniforms.f[24].wwww;
    reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
    reg_tmp3 = uniforms.f[22];
    reg_tmp2 = uniforms.f[23] + -reg_tmp3;
    reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
    if (uniforms.b[6]) {
        sub_267_268();
    }
    reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_267_268() {
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
    return false;
}
bool sub_279_288() {
    reg_tmp5.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
    reg_tmp5.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
    reg_tmp5.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
    reg_tmp4.x = dot_s(reg_tmp5.xyzz, reg_tmp5.xyzz);
    reg_tmp4.x = rsq_s(reg_tmp4.x);
    reg_tmp4.xyz = (mul_s(reg_tmp5.xyzz, reg_tmp4.xxxx)).xyz;
    reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
    reg_tmp12 = uniforms.f[94].zzzz + reg_tmp4;
    vs_out_attr5 = reg_tmp12;
    return false;
}
bool sub_321_329() {
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_323_324();
    } else {
        sub_324_328();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_323_324() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_324_328() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_325_326();
    } else {
        sub_326_327();
    }
    return false;
}
bool sub_325_326() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_326_327() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_329_333() {
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_333_4096() {
    {
        sub_37_197();
    }
    {
        sub_197_214();
    }
    {
        sub_339_366();
    }
    {
        sub_279_288();
    }
    return true;
}
bool sub_339_366() {
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    if (uniforms.b[9]) {
        sub_341_346();
    } else {
        sub_346_365();
    }
    return false;
}
bool sub_341_346() {
    {
        sub_321_329();
    }
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_346_365() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    if (all(not(conditional_code))) {
        sub_349_355();
    } else {
        sub_355_364();
    }
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_349_355() {
    reg_tmp6 = reg_tmp10;
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_355_364() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_356_360();
    } else {
        sub_360_363();
    }
    return false;
}
bool sub_356_360() {
    {
        sub_366_373();
    }
    reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
    reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
    reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
    return false;
}
bool sub_360_363() {
    {
        sub_329_333();
    }
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_366_373() {
    reg_tmp2 = -reg_tmp15;
    reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
    reg_tmp2.w = rsq_s(reg_tmp2.w);
    reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
    reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
// reference: EB3EB7679F12CFDA, B6A06BC7A678BBFB
// shader: 8B30, EA0B5E689BDE56E5

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor1.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
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
// reference: FB35E2F4B7D17575, EA0B5E689BDE56E5
// program: B6A06BC7A678BBFB, D0BDF6740B5CE976, EA0B5E689BDE56E5
// reference: F98B188944AB6E9D, B6A06BC7A678BBFB
// shader: 8B30, 15ACC04977614EE2

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
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
// reference: FB35E2F47BA2E1ED, 15ACC04977614EE2
// program: B6A06BC7A678BBFB, D0BDF6740B5CE976, 15ACC04977614EE2
// shader: 8B31, C314540E9C5CE4AD

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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
bool sub_34_214();
bool sub_40_146();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_144_145();
bool sub_146_213();
bool sub_149_159();
bool sub_159_164();
bool sub_165_177();
bool sub_177_202();
bool sub_178_196();
bool sub_196_201();
bool sub_211_212();
bool sub_214_231();
bool sub_231_307();
bool sub_270_290();
bool sub_271_276();
bool sub_276_289();
bool sub_280_283();
bool sub_283_288();
bool sub_290_303();
bool sub_291_296();
bool sub_296_302();
bool sub_307_330();
bool sub_330_357();
bool sub_332_337();
bool sub_337_356();
bool sub_340_346();
bool sub_346_355();
bool sub_347_351();
bool sub_351_354();
bool sub_357_365();
bool sub_359_360();
bool sub_360_364();
bool sub_361_362();
bool sub_362_363();
bool sub_365_372();
bool sub_372_376();
bool sub_376_400();
bool sub_378_382();
bool sub_382_399();
bool sub_383_397();
bool sub_386_393();
bool sub_393_396();
bool sub_397_398();
bool sub_400_416();
bool sub_402_406();
bool sub_406_415();
bool sub_407_413();
bool sub_413_414();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    {
        sub_34_214();
    }
    {
        sub_307_330();
    }
    {
        sub_330_357();
    }
    {
        sub_376_400();
    }
    {
        sub_400_416();
    }
    return true;
}
bool sub_7_12() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp5.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp5.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_214() {
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    if (uniforms.b[1]) {
        sub_40_146();
    } else {
        sub_146_213();
    }
    return false;
}
bool sub_40_146() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    reg_tmp7 = uniforms.f[93].xxxx;
    reg_tmp12 = uniforms.f[93].xxxx;
    reg_tmp11 = uniforms.f[93].xxxx;
    reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    vs_out_attr2 = -reg_tmp15;
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_144_145();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_47_76() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_12_21();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_12_21();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_12_21();
    }
    if (uniforms.b[8]) {
        sub_58_62();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_214_231();
    }
    return false;
}
bool sub_58_62() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_12_21();
    }
    return false;
}
bool sub_76_135() {
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    return false;
}
bool sub_77_109() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_21_34();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_21_34();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_21_34();
    }
    if (uniforms.b[8]) {
        sub_88_92();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    {
        sub_231_307();
    }
    return false;
}
bool sub_88_92() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_21_34();
    }
    return false;
}
bool sub_109_134() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_7_12();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_7_12();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_7_12();
    }
    if (uniforms.b[8]) {
        sub_120_124();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_120_124() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_7_12();
    }
    return false;
}
bool sub_144_145() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_146_213() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    if (uniforms.b[2]) {
        sub_149_159();
    } else {
        sub_159_164();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_165_177();
    } else {
        sub_177_202();
    }
    vs_out_attr2 = -reg_tmp15;
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_211_212();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_149_159() {
    reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp7.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp7.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp7.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_159_164() {
    address_registers.x = (ivec2(uniforms.f[93].xx)).x;
    reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
    reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
    reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_165_177() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    {
        sub_214_231();
    }
    return false;
}
bool sub_177_202() {
    if (all(conditional_code)) {
        sub_178_196();
    } else {
        sub_196_201();
    }
    return false;
}
bool sub_178_196() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp11.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp11.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp13.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    {
        sub_231_307();
    }
    return false;
}
bool sub_196_201() {
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_211_212() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_214_231() {
    uint jmp_to = 214u;
    while (true) {
        switch (jmp_to) {
        case 214u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 230u; break; }
            }
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
            conditional_code = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
            if (conditional_code.x) {
                { jmp_to = 230u; break; }
            }
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 230u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_231_307() {
    uint jmp_to = 231u;
    while (true) {
        switch (jmp_to) {
        case 231u: {
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            reg_tmp0 = uniforms.f[93].yxxx;
            if (!uniforms.b[15]) {
                { jmp_to = 306u; break; }
            }
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
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
            conditional_code = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            if (!conditional_code.x) {
                { jmp_to = 268u; break; }
            }
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            reg_tmp7.w = (reg_tmp6).w;
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            if (uniforms.b[0]) {
                { jmp_to = 306u; break; }
            }
        }
        case 268u: {
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            if (conditional_code.x) {
                sub_270_290();
            } else {
                sub_290_303();
            }
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 306u: {
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_270_290() {
    if (conditional_code.y) {
        sub_271_276();
    } else {
        sub_276_289();
    }
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_271_276() {
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_276_289() {
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    if (conditional_code.x) {
        sub_280_283();
    } else {
        sub_283_288();
    }
    return false;
}
bool sub_280_283() {
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_283_288() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_290_303() {
    if (conditional_code.y) {
        sub_291_296();
    } else {
        sub_296_302();
    }
    return false;
}
bool sub_291_296() {
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_296_302() {
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_307_330() {
    reg_tmp1.xyz = (uniforms.f[93].xxyx).xyz;
    reg_tmp8.x = dot_3(uniforms.f[0].xyz, reg_tmp12.xyz);
    reg_tmp8.y = dot_3(uniforms.f[1].xyz, reg_tmp12.xyz);
    reg_tmp8.z = dot_3(uniforms.f[2].xyz, reg_tmp12.xyz);
    reg_tmp14.x = dot_3(uniforms.f[90].xyz, reg_tmp8.xyz);
    reg_tmp14.y = dot_3(uniforms.f[91].xyz, reg_tmp8.xyz);
    reg_tmp14.z = dot_3(uniforms.f[92].xyz, reg_tmp8.xyz);
    reg_tmp14.w = (uniforms.f[93].xxxx).w;
    reg_tmp8 = reg_tmp14;
    reg_tmp14.x = dot_s(reg_tmp8, reg_tmp8);
    reg_tmp14.x = rsq_s(reg_tmp14.x);
    reg_tmp14 = mul_s(reg_tmp8, reg_tmp14.xxxx);
    reg_tmp9.x = dot_3(reg_tmp14.xyz, reg_tmp1.xyz);
    reg_tmp9.x = (max(uniforms.f[93].xxxx, reg_tmp9.xxxx)).x;
    reg_tmp9.x = (min(uniforms.f[93].yyyy, reg_tmp9.xxxx)).x;
    reg_tmp9.x = (uniforms.f[93].yyyy + -reg_tmp9.xxxx).x;
    reg_tmp9.x = log2(reg_tmp9.x);
    reg_tmp9.x = (mul_s(uniforms.f[85].xxxx, reg_tmp9.xxxx)).x;
    reg_tmp9.x = exp2(reg_tmp9.x);
    reg_tmp2.w = (mul_s(uniforms.f[85].yyyy, reg_tmp9.xxxx)).w;
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    reg_tmp2.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    vs_out_attr3 = reg_tmp2;
    return false;
}
bool sub_330_357() {
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    if (uniforms.b[9]) {
        sub_332_337();
    } else {
        sub_337_356();
    }
    return false;
}
bool sub_332_337() {
    {
        sub_357_365();
    }
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_337_356() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    if (all(not(conditional_code))) {
        sub_340_346();
    } else {
        sub_346_355();
    }
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_340_346() {
    reg_tmp6 = reg_tmp10;
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_346_355() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_347_351();
    } else {
        sub_351_354();
    }
    return false;
}
bool sub_347_351() {
    {
        sub_365_372();
    }
    reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
    reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
    reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
    return false;
}
bool sub_351_354() {
    {
        sub_372_376();
    }
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_357_365() {
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_359_360();
    } else {
        sub_360_364();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_359_360() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_360_364() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_361_362();
    } else {
        sub_362_363();
    }
    return false;
}
bool sub_361_362() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_362_363() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_365_372() {
    reg_tmp2 = -reg_tmp15;
    reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
    reg_tmp2.w = rsq_s(reg_tmp2.w);
    reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
    reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_372_376() {
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_376_400() {
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    if (uniforms.b[10]) {
        sub_378_382();
    } else {
        sub_382_399();
    }
    return false;
}
bool sub_378_382() {
    {
        sub_357_365();
    }
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_382_399() {
    if (uniforms.b[13]) {
        sub_383_397();
    } else {
        sub_397_398();
    }
    return false;
}
bool sub_383_397() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    if (all(not(conditional_code))) {
        sub_386_393();
    } else {
        sub_393_396();
    }
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_386_393() {
    reg_tmp6 = reg_tmp10;
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
    reg_tmp6.w = rcp_s(reg_tmp4.z);
    reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_393_396() {
    {
        sub_372_376();
    }
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    return false;
}
bool sub_397_398() {
    vs_out_attr5 = uniforms.f[93].xxxx;
    return false;
}
bool sub_400_416() {
    reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
    if (uniforms.b[11]) {
        sub_402_406();
    } else {
        sub_406_415();
    }
    return false;
}
bool sub_402_406() {
    {
        sub_357_365();
    }
    reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_406_415() {
    if (uniforms.b[14]) {
        sub_407_413();
    } else {
        sub_413_414();
    }
    return false;
}
bool sub_407_413() {
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    reg_tmp5.zw = (reg_tmp6.zwww).zw;
    {
        sub_372_376();
    }
    reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_413_414() {
    vs_out_attr6 = uniforms.f[93].xxxx;
    return false;
}
// reference: 1F9C2FFD98F835B4, C314540E9C5CE4AD
// shader: 8B30, 5427C73FFAC43C9B

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
refl_value.r = (lut_scale_rr * LookupLightingLUTSigned(6, dot(light_vector, normal)));
refl_value.g = (lut_scale_rg * LookupLightingLUTSigned(5, dot(light_vector, normal)));
refl_value.b = (lut_scale_rb * LookupLightingLUTSigned(4, dot(light_vector, normal)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
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
// reference: 59FA65499FEF8E4A, 5427C73FFAC43C9B
// program: C314540E9C5CE4AD, B80FA1B7F97410CD, 5427C73FFAC43C9B
// reference: 0D298013434194F3, C314540E9C5CE4AD
// reference: 84F95D84698D2E32, 9583E5EAA2BC5470
// reference: 964CF26AB2348F75, 9583E5EAA2BC5470
// shader: 8B30, EA04FA07E6C35081

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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[2].position + view);
spot_dir = light_src[2].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[2].diffuse * dot_product) + light_src[2].ambient) * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[2].specular_0) + (light_src[2].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(18, clamp(light_src[2].dist_atten_scale * length(-view - light_src[2].position) + light_src[2].dist_atten_bias, 0.0, 1.0)) * 1.0;
light_vector = normalize(light_src[3].position + view);
spot_dir = light_src[3].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[3].diffuse * dot_product) + light_src[3].ambient) * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[3].specular_0) + (light_src[3].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(19, clamp(light_src[3].dist_atten_scale * length(-view - light_src[3].position) + light_src[3].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((secondary_fragment_color.r) * (1.0 - const_color[0].a) + (secondary_fragment_color.b) * (1.0 - (1.0 - const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (texcolor0.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.ggg) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((rounded_primary_color.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2C7FE14E50DA0700, EA04FA07E6C35081
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, EA04FA07E6C35081
// reference: 6035BCD83F01F2B5, 1993E2D62BB91686
// reference: 72801336E4B853F2, 1993E2D62BB91686
// reference: FD2C7FB8F7D157B6, B6A06BC7A678BBFB
// reference: EF99D0562C68F6F1, B6A06BC7A678BBFB
// reference: 59FA6549ED5F5786, 5427C73FFAC43C9B
// reference: 2C7FE14E226ADECC, EA04FA07E6C35081
// shader: 8B31, C017E3232EE056B5

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

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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
bool sub_6_11();
bool sub_11_20();
bool sub_20_29();
bool sub_37_197();
bool sub_42_139();
bool sub_48_76();
bool sub_59_63();
bool sub_76_129();
bool sub_77_104();
bool sub_88_92();
bool sub_104_128();
bool sub_115_119();
bool sub_137_138();
bool sub_139_194();
bool sub_142_152();
bool sub_152_157();
bool sub_158_168();
bool sub_168_184();
bool sub_169_179();
bool sub_179_183();
bool sub_192_193();
bool sub_197_214();
bool sub_203_208();
bool sub_205_206();
bool sub_212_213();
bool sub_214_260();
bool sub_218_259();
bool sub_223_225();
bool sub_225_249();
bool sub_228_233();
bool sub_239_248();
bool sub_243_244();
bool sub_244_247();
bool sub_251_258();
bool sub_260_270();
bool sub_267_268();
bool sub_270_279();
bool sub_297_321();
bool sub_299_303();
bool sub_303_320();
bool sub_304_318();
bool sub_307_314();
bool sub_314_317();
bool sub_318_319();
bool sub_321_329();
bool sub_323_324();
bool sub_324_328();
bool sub_325_326();
bool sub_326_327();
bool sub_329_333();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    {
        sub_37_197();
    }
    {
        sub_197_214();
    }
    {
        sub_270_279();
    }
    {
        sub_297_321();
    }
    return true;
}
bool sub_6_11() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_11_20() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_20_29() {
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp3.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp3.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp3.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp4.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp4.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_37_197() {
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    if (uniforms.b[1]) {
        sub_42_139();
    } else {
        sub_139_194();
    }
    vs_out_attr1 = uniforms.f[93].xxxx;
    vs_out_attr2 = uniforms.f[93].xxxx;
    return false;
}
bool sub_42_139() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    reg_tmp7 = uniforms.f[93].xxxx;
    reg_tmp12 = uniforms.f[93].xxxx;
    reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_48_76();
    } else {
        sub_76_129();
    }
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_137_138();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_48_76() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_11_20();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_11_20();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_11_20();
    }
    if (uniforms.b[8]) {
        sub_59_63();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_59_63() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_11_20();
    }
    return false;
}
bool sub_76_129() {
    if (all(conditional_code)) {
        sub_77_104();
    } else {
        sub_104_128();
    }
    return false;
}
bool sub_77_104() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_20_29();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_20_29();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_20_29();
    }
    if (uniforms.b[8]) {
        sub_88_92();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_88_92() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_20_29();
    }
    return false;
}
bool sub_104_128() {
    conditional_code = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    {
        sub_6_11();
    }
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    {
        sub_6_11();
    }
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    if (conditional_code.x) {
        sub_6_11();
    }
    if (uniforms.b[8]) {
        sub_115_119();
    }
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_115_119() {
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    if (conditional_code.y) {
        sub_6_11();
    }
    return false;
}
bool sub_137_138() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_139_194() {
    reg_tmp0 = uniforms.f[7];
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
    if (uniforms.b[2]) {
        sub_142_152();
    } else {
        sub_152_157();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_158_168();
    } else {
        sub_168_184();
    }
    reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
    reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
    reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
    reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
    reg_tmp1.x = (-reg_tmp0.wwww).x;
    reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
    conditional_code.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
    conditional_code.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
    if (all(conditional_code)) {
        sub_192_193();
    }
    vs_out_attr0 = reg_tmp0;
    return false;
}
bool sub_142_152() {
    reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    reg_tmp7.x = dot_s(uniforms.f[25 + address_registers.x], reg_tmp15);
    reg_tmp7.y = dot_s(uniforms.f[26 + address_registers.x], reg_tmp15);
    reg_tmp7.z = dot_s(uniforms.f[27 + address_registers.x], reg_tmp15);
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_152_157() {
    address_registers.x = (ivec2(uniforms.f[93].xx)).x;
    reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
    reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
    reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_158_168() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_168_184() {
    if (all(conditional_code)) {
        sub_169_179();
    } else {
        sub_179_183();
    }
    return false;
}
bool sub_169_179() {
    reg_tmp12.x = dot_3(uniforms.f[25 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.y = dot_3(uniforms.f[26 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp12.z = dot_3(uniforms.f[27 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_179_183() {
    reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
    reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
    reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_192_193() {
    reg_tmp0.x = (-reg_tmp0.wwww).x;
    return false;
}
bool sub_197_214() {
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    conditional_code = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    if (conditional_code.y) {
        sub_203_208();
    }
    if (uniforms.b[12]) {
        sub_214_260();
    }
    if (uniforms.b[5]) {
        sub_260_270();
    }
    conditional_code = equal(uniforms.f[93].xx, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_212_213();
    }
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_203_208() {
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    if (uniforms.b[7]) {
        sub_205_206();
    }
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_205_206() {
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_212_213() {
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_214_260() {
    reg_tmp1 = uniforms.f[20];
    reg_tmp2 = uniforms.f[21];
    reg_tmp3 = uniforms.f[93].xxxx;
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop217 = 0u; loop217 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop217) {
        sub_218_259();
    }
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_218_259() {
    address_registers.x = (ivec2(reg_tmp3.xy)).x;
    reg_tmp4.x = (uniforms.f[81 + address_registers.x].wwww).x;
    reg_tmp4.y = (uniforms.f[83 + address_registers.x].wwww).y;
    conditional_code = equal(uniforms.f[93].xy, reg_tmp4.xy);
    if (conditional_code.x) {
        sub_223_225();
    } else {
        sub_225_249();
    }
    conditional_code.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
    conditional_code.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
    if (conditional_code.y) {
        sub_251_258();
    }
    reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
    return false;
}
bool sub_223_225() {
    reg_tmp6.x = dot_3(uniforms.f[81 + address_registers.x].xyz, reg_tmp14.xyz);
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    return false;
}
bool sub_225_249() {
    reg_tmp4 = uniforms.f[81 + address_registers.x] + -reg_tmp15;
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    if (conditional_code.y) {
        sub_228_233();
    }
    reg_tmp5 = uniforms.f[82 + address_registers.x];
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.ww);
    reg_tmp4.w = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp4.w = rsq_s(reg_tmp4.w);
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp4.wwww);
    if (conditional_code.x) {
        sub_239_248();
    }
    reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp4.xyz);
    return false;
}
bool sub_228_233() {
    reg_tmp5.x = (uniforms.f[93].yyyy).x;
    reg_tmp5.z = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
    reg_tmp5.y = (mul_s(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
    reg_tmp6.y = dot_3(uniforms.f[83 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.y = rcp_s(reg_tmp6.y);
    return false;
}
bool sub_239_248() {
    reg_tmp5.x = dot_3(uniforms.f[82 + address_registers.x].xyz, -reg_tmp4.xyz);
    reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx, uniforms.f[84 + address_registers.x].yyyy))).y;
    conditional_code = equal(uniforms.f[93].yy, reg_tmp5.xy);
    if (conditional_code.y) {
        sub_243_244();
    } else {
        sub_244_247();
    }
    reg_tmp6.y = (mul_s(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
    return false;
}
bool sub_243_244() {
    reg_tmp5.x = (uniforms.f[93].xxxx).x;
    return false;
}
bool sub_244_247() {
    reg_tmp5.y = log2(reg_tmp5.x);
    reg_tmp5.y = (mul_s(uniforms.f[84 + address_registers.x].xxxx, reg_tmp5.yyyy)).y;
    reg_tmp5.x = exp2(reg_tmp5.y);
    return false;
}
bool sub_251_258() {
    reg_tmp6.x = (max(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
    reg_tmp9.xyz = (fma_s(reg_tmp1.xyzz, uniforms.f[79 + address_registers.x].xyzz, reg_tmp9.xyzz)).xyz;
    reg_tmp4 = mul_s(uniforms.f[80 + address_registers.x], reg_tmp2);
    reg_tmp5.xyz = (mul_s(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
    reg_tmp5.xyz = (mul_s(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
    reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
    reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
    return false;
}
bool sub_260_270() {
    reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
    reg_tmp2 = uniforms.f[24].wwww;
    reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
    reg_tmp3 = uniforms.f[22];
    reg_tmp2 = uniforms.f[23] + -reg_tmp3;
    reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
    if (uniforms.b[6]) {
        sub_267_268();
    }
    reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_267_268() {
    reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
    return false;
}
bool sub_270_279() {
    reg_tmp5.x = dot_3(uniforms.f[90].xyz, reg_tmp12.xyz);
    reg_tmp5.y = dot_3(uniforms.f[91].xyz, reg_tmp12.xyz);
    reg_tmp5.z = dot_3(uniforms.f[92].xyz, reg_tmp12.xyz);
    reg_tmp4.x = dot_s(reg_tmp5.xyzz, reg_tmp5.xyzz);
    reg_tmp4.x = rsq_s(reg_tmp4.x);
    reg_tmp4.xyz = (mul_s(reg_tmp5.xyzz, reg_tmp4.xxxx)).xyz;
    reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
    reg_tmp12 = uniforms.f[94].zzzz + reg_tmp4;
    vs_out_attr4 = reg_tmp12;
    return false;
}
bool sub_297_321() {
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    if (uniforms.b[10]) {
        sub_299_303();
    } else {
        sub_303_320();
    }
    return false;
}
bool sub_299_303() {
    {
        sub_321_329();
    }
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_303_320() {
    if (uniforms.b[13]) {
        sub_304_318();
    } else {
        sub_318_319();
    }
    return false;
}
bool sub_304_318() {
    conditional_code = equal(uniforms.f[95].xy, reg_tmp0.xy);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    if (all(not(conditional_code))) {
        sub_307_314();
    } else {
        sub_314_317();
    }
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_307_314() {
    reg_tmp6 = reg_tmp10;
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
    reg_tmp6.w = rcp_s(reg_tmp4.z);
    reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_314_317() {
    {
        sub_329_333();
    }
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    return false;
}
bool sub_318_319() {
    vs_out_attr5 = uniforms.f[93].xxxx;
    return false;
}
bool sub_321_329() {
    conditional_code = equal(uniforms.f[93].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_323_324();
    } else {
        sub_324_328();
    }
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_323_324() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_324_328() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_325_326();
    } else {
        sub_326_327();
    }
    return false;
}
bool sub_325_326() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_326_327() {
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_329_333() {
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
// reference: EF99D0560229AE45, C017E3232EE056B5
// shader: 8B30, AD586C968DB2E9BA

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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
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
// reference: 3F41287ADE540368, AD586C968DB2E9BA
// program: C017E3232EE056B5, D0BDF6740B5CE976, AD586C968DB2E9BA
// reference: D4FED0A637ABB5AA, 5D82DD64D93FB5CE
// reference: F58F6E05A78570EC, 4E545413B902FBCE
// reference: A1AF448FF9C72878, 9583E5EAA2BC5470
// reference: 0EEBD6C83841945A, DD1B5AA99EDA2A36
// reference: 0EEBD6C8D778071B, DD1B5AA99EDA2A36
// reference: A1AF448F16FEBB39, 9583E5EAA2BC5470
// program: 0000000000000000, 0000000000000000, 0BF1EB09F86FE320
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 1C81792A8869223A
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, C18D3D2A00004367
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, FF1B1B7D4B7919D0
// program: 4E545413B902FBCE, D0BDF6740B5CE976, 74210693C7372778
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 4E8B40EC05E52645
// program: 1993E2D62BB91686, B80FA1B7F97410CD, C507F6CE9C927C7B
// program: 0000000000000000, 0000000000000000, 05ACA9E48DF2EA73
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 6C4F6E107EFBE36B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, B50DFB3C2276697B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 5D82DD64D93FB5CE
// program: B6A06BC7A678BBFB, D0BDF6740B5CE976, EA0B5E689BDE56E5
// program: B6A06BC7A678BBFB, D0BDF6740B5CE976, 15ACC04977614EE2
// program: C314540E9C5CE4AD, B80FA1B7F97410CD, 5427C73FFAC43C9B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, EA04FA07E6C35081
// program: C017E3232EE056B5, D0BDF6740B5CE976, AD586C968DB2E9BA
// program: 0000000000000000, 0000000000000000, 0BF1EB09F86FE320
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 1C81792A8869223A
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, C18D3D2A00004367
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, FF1B1B7D4B7919D0
// program: 4E545413B902FBCE, D0BDF6740B5CE976, 74210693C7372778
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 4E8B40EC05E52645
// program: 1993E2D62BB91686, B80FA1B7F97410CD, C507F6CE9C927C7B
// program: 0000000000000000, 0000000000000000, 05ACA9E48DF2EA73
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 6C4F6E107EFBE36B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, B50DFB3C2276697B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 5D82DD64D93FB5CE
// program: B6A06BC7A678BBFB, D0BDF6740B5CE976, EA0B5E689BDE56E5
// program: B6A06BC7A678BBFB, D0BDF6740B5CE976, 15ACC04977614EE2
// program: C314540E9C5CE4AD, B80FA1B7F97410CD, 5427C73FFAC43C9B
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, EA04FA07E6C35081
// program: C017E3232EE056B5, D0BDF6740B5CE976, AD586C968DB2E9BA
// program: 0000000000000000, 0000000000000000, 0BF1EB09F86FE320
// reference: 5A3F863CFFC9B7D9, 4E545413B902FBCE
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, 1C81792A8869223A
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, C18D3D2A00004367
// program: 9583E5EAA2BC5470, D0BDF6740B5CE976, FF1B1B7D4B7919D0
// program: 4E545413B902FBCE, D0BDF6740B5CE976, 74210693C7372778
// reference: 3E3C36C720473F21, 08BCAB0557C02D18
// reference: 805E3AB5014EB65E, 9583E5EAA2BC5470

// shader: 8B31, BA674B8B6FF1339A

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
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;

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

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp5.x = dot_s(uniforms.f[7], vs_in_reg0);
    reg_tmp5.y = dot_s(uniforms.f[8], vs_in_reg0);
    reg_tmp5.z = dot_s(uniforms.f[9], vs_in_reg0);
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    reg_tmp6.x = dot_s(uniforms.f[4], reg_tmp5);
    reg_tmp6.y = dot_s(uniforms.f[5], reg_tmp5);
    reg_tmp6.z = dot_s(uniforms.f[6], reg_tmp5);
    reg_tmp6.w = (uniforms.f[91].yyyy).w;
    reg_tmp5.x = dot_s(uniforms.f[0], reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[1], reg_tmp6);
    reg_tmp5.z = dot_s(uniforms.f[2], reg_tmp6);
    reg_tmp5.w = dot_s(uniforms.f[3], reg_tmp6);
    vs_out_attr0 = reg_tmp5;
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    reg_tmp7.xy = (uniforms.f[10].wwww).xy;
    reg_tmp7.x = (-uniforms.f[92].zzzz + reg_tmp7.xxxx).x;
    reg_tmp7.xy = (mul_s(uniforms.f[18].xyyy, reg_tmp7.xyyy)).xy;
    reg_tmp5 = vs_in_reg4;
    reg_tmp5.xy = (reg_tmp5.xyyy + reg_tmp7.xyyy).xy;
    vs_out_attr2.x = dot_s(uniforms.f[12], reg_tmp5);
    vs_out_attr2.y = dot_s(uniforms.f[13], reg_tmp5);
    vs_out_attr2.z = (reg_tmp5.zzzz).z;
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    reg_tmp5 = vs_in_reg5;
    vs_out_attr3.x = dot_s(uniforms.f[14], reg_tmp5);
    vs_out_attr3.y = dot_s(uniforms.f[15], reg_tmp5);
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    return true;
}
// reference: B9420B488EDBFAA6, BA674B8B6FF1339A
// shader: 8DD9, 5ADC687F067D9F6C

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
    primary_color = min(abs(vtx_color), vec4(1.0));

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
// reference: 83ABF08ECCF76B42, 5ADC687F067D9F6C
// shader: 8B30, 53598CBDFE71DB11
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3F41287A86E18D59, 53598CBDFE71DB11
// program: BA674B8B6FF1339A, 5ADC687F067D9F6C, 53598CBDFE71DB11
// shader: 8B30, 22E0D74A8B55120D
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2584B8EB081287D7, 22E0D74A8B55120D
// program: BA674B8B6FF1339A, 5ADC687F067D9F6C, 22E0D74A8B55120D
// reference: F716F4080D2E9CD4, BA674B8B6FF1339A
// shader: 8B31, 6FDD6A1C99DF310F

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
layout(location = 4) in vec4 vs_in_reg4;

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

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp5.x = dot_s(uniforms.f[7], vs_in_reg0);
    reg_tmp5.y = dot_s(uniforms.f[8], vs_in_reg0);
    reg_tmp5.z = dot_s(uniforms.f[9], vs_in_reg0);
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    reg_tmp6.x = dot_s(uniforms.f[4], reg_tmp5);
    reg_tmp6.y = dot_s(uniforms.f[5], reg_tmp5);
    reg_tmp6.z = dot_s(uniforms.f[6], reg_tmp5);
    reg_tmp6.w = (uniforms.f[91].yyyy).w;
    reg_tmp5.x = dot_s(uniforms.f[0], reg_tmp6);
    reg_tmp5.y = dot_s(uniforms.f[1], reg_tmp6);
    reg_tmp5.z = dot_s(uniforms.f[2], reg_tmp6);
    reg_tmp5.w = dot_s(uniforms.f[3], reg_tmp6);
    vs_out_attr0 = reg_tmp5;
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    vs_out_attr2.zw = (uniforms.f[91].xxxx).zw;
    reg_tmp5.xy = (uniforms.f[92].zzzz).xy;
    reg_tmp5.y = (reg_tmp5.yyyy + vs_in_reg4.zzzz).y;
    vs_out_attr3.x = dot_s(uniforms.f[14], reg_tmp5);
    vs_out_attr3.y = dot_s(uniforms.f[15], reg_tmp5);
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    return true;
}
// reference: F716F4084CF04846, 6FDD6A1C99DF310F
// shader: 8DD9, 13A54CCD8AA1DDA2

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
    primary_color = min(abs(vtx_color), vec4(1.0));

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
// reference: A55C6948CCF76B42, 13A54CCD8AA1DDA2
// shader: 8B30, B817BE8F9AF40552
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) * (texcolor1.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((texcolor0.g) * (texcolor1.g) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.b) * (texcolor1.b) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_3 = (rounded_primary_color.rgb);
float alpha_output_3 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3 * 2.0, alpha_output_3 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F138ED2A1182AA30, B817BE8F9AF40552
// program: 6FDD6A1C99DF310F, 13A54CCD8AA1DDA2, B817BE8F9AF40552
// shader: 8B30, 4B3E71DD2B012BDE
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F51696E770CCA444, 4B3E71DD2B012BDE
// program: BA674B8B6FF1339A, 5ADC687F067D9F6C, 4B3E71DD2B012BDE
// shader: 8B31, 3E3F62D2D8C36EC0

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

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp7.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.xyyy)).xy;
    address_registers.xy = ivec2(reg_tmp7.xy);
    reg_tmp5.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    reg_tmp5.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    reg_tmp5.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp5 = mul_s(reg_tmp5, vs_in_reg7.xxxx);
    reg_tmp6 = mul_s(reg_tmp6, vs_in_reg7.xxxx);
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.yyyy, reg_tmp5);
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp6 = fma_s(reg_tmp7, vs_in_reg7.yyyy, reg_tmp6);
    reg_tmp7.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.zwww)).xy;
    address_registers.xy = ivec2(reg_tmp7.xy);
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.zzzz, reg_tmp5);
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp6 = fma_s(reg_tmp7, vs_in_reg7.zzzz, reg_tmp6);
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    reg_tmp8 = fma_s(reg_tmp7, vs_in_reg7.wwww, reg_tmp5);
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.wwww, reg_tmp6);
    reg_tmp8.w = (uniforms.f[91].yyyy).w;
    reg_tmp5.w = (uniforms.f[91].xxxx).w;
    reg_tmp9.x = dot_s(reg_tmp5, reg_tmp5);
    reg_tmp9.x = rsq_s(reg_tmp9.x);
    reg_tmp9 = mul_s(reg_tmp5, reg_tmp9.xxxx);
    reg_tmp5.x = dot_s(uniforms.f[7], reg_tmp8);
    reg_tmp5.y = dot_s(uniforms.f[8], reg_tmp8);
    reg_tmp5.z = dot_s(uniforms.f[9], reg_tmp8);
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp5);
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp5);
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp5);
    reg_tmp1.w = (-uniforms.f[91].yyyy).w;
    vs_out_attr5 = reg_tmp1;
    reg_tmp5.x = dot_3(uniforms.f[7].xyz, reg_tmp9.xyz);
    reg_tmp5.y = dot_3(uniforms.f[8].xyz, reg_tmp9.xyz);
    reg_tmp5.z = dot_3(uniforms.f[9].xyz, reg_tmp9.xyz);
    reg_tmp6.x = dot_3(uniforms.f[4].xyz, reg_tmp5.xyz);
    reg_tmp6.y = dot_3(uniforms.f[5].xyz, reg_tmp5.xyz);
    reg_tmp6.z = dot_3(uniforms.f[6].xyz, reg_tmp5.xyz);
    reg_tmp6.w = (uniforms.f[91].xxxx).w;
    reg_tmp2.x = dot_s(reg_tmp6, reg_tmp6);
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    reg_tmp2 = mul_s(reg_tmp6, reg_tmp2.xxxx);
    conditional_code = equal(-uniforms.f[91].yy, reg_tmp2.zz);
    reg_tmp3 = uniforms.f[91].yyyy + reg_tmp2.zzzz;
    reg_tmp3 = mul_s(uniforms.f[92].zzzz, reg_tmp3);
    vs_out_attr6.w = (uniforms.f[91].xxxx).w;
    reg_tmp3 = vec4(rsq_s(reg_tmp3.x));
    reg_tmp4 = mul_s(uniforms.f[92].zzzz, reg_tmp2);
    if (!conditional_code.x) {
        sub_1();
    } else {
        sub_2();
    }
    reg_tmp5.x = dot_s(uniforms.f[0], -reg_tmp1);
    reg_tmp5.y = dot_s(uniforms.f[1], -reg_tmp1);
    reg_tmp5.z = dot_s(uniforms.f[2], -reg_tmp1);
    reg_tmp5.w = dot_s(uniforms.f[3], -reg_tmp1);
    vs_out_attr0 = reg_tmp5;
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    vs_out_attr4.zw = (uniforms.f[91].xxxx).zw;
    return true;
}
bool sub_1() {
    vs_out_attr6.z = rcp_s(reg_tmp3.x);
    vs_out_attr6.xy = (mul_s(reg_tmp4, reg_tmp3)).xy;
    return false;
}
bool sub_2() {
    vs_out_attr6.x = (uniforms.f[91].yyyy).x;
    vs_out_attr6.y = (uniforms.f[91].xxxx).y;
    vs_out_attr6.z = (uniforms.f[91].xxxx).z;
    return false;
}
// reference: C022612429CAC9C7, 3E3F62D2D8C36EC0
// shader: 8DD9, 1CB2FE196A34ED92

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
in vec4 vs_out_attr6[];
struct Vertex {
    vec4 attributes[7];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[6].x, vtx.attributes[6].y, vtx.attributes[6].z, vtx.attributes[6].w);
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

    texcoord0_w = vtx.attributes[2].z;
    view = vec3(vtx.attributes[5].x, vtx.attributes[5].y, vtx.attributes[5].z);
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
    prim_buffer[0].attributes = vec4[7](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0], vs_out_attr6[0]);
    prim_buffer[1].attributes = vec4[7](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1], vs_out_attr6[1]);
    prim_buffer[2].attributes = vec4[7](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2], vs_out_attr6[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: B5372A1237E9B366, 1CB2FE196A34ED92
// program: 3E3F62D2D8C36EC0, 1CB2FE196A34ED92, 4B3E71DD2B012BDE
// shader: 8B30, D07C0125AD3F01B9
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F51696E7173B3032, D07C0125AD3F01B9
// program: BA674B8B6FF1339A, 5ADC687F067D9F6C, D07C0125AD3F01B9
// reference: B9420B48CF052E34, 6FDD6A1C99DF310F
// shader: 8B30, 46D83C69F0E9A79E
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
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp(min((primary_fragment_color.a) + (secondary_fragment_color.a), 1.0) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AB02540B4C3B45D8, 46D83C69F0E9A79E
// program: 3E3F62D2D8C36EC0, 1CB2FE196A34ED92, 46D83C69F0E9A79E
// shader: 8B30, 3B8212DC1038224A
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
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(normal, normalize(half_vector)))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp(min((primary_fragment_color.a) + (secondary_fragment_color.a), 1.0) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 93C62DEB0C139CB9, 3B8212DC1038224A
// program: 3E3F62D2D8C36EC0, 1CB2FE196A34ED92, 3B8212DC1038224A
// reference: 4C2EB72ACDE390F8, 46D83C69F0E9A79E
// reference: BAAADF160C139CB9, 3B8212DC1038224A
// shader: 8B30, 0E284A68CC1A6315
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 17C9C61A47CD60EF, 0E284A68CC1A6315
// program: 0000000000000000, 0000000000000000, 0E284A68CC1A6315
// shader: 8B30, 1AFB8DE7813BC51D
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2584B8EB381FA280, 1AFB8DE7813BC51D
// program: BA674B8B6FF1339A, 5ADC687F067D9F6C, 1AFB8DE7813BC51D
// reference: 8E769E64AA3FAFB5, 3E3F62D2D8C36EC0
// reference: C31B9ABFCDE390F8, 3B8212DC1038224A
// reference: B9420B480D2E9CD4, BA674B8B6FF1339A
// reference: 8E769E64AA3FAFB5, 3E3F62D2D8C36EC0
// reference: 17C9C61A47CD60EF, 0E284A68CC1A6315
// reference: C31B9ABFCDE390F8, 3B8212DC1038224A
// reference: 93C62DEB0C139CB9, 3B8212DC1038224A
// reference: AB02540B4C3B45D8, 46D83C69F0E9A79E
// reference: 4C2EB72ACDE390F8, 46D83C69F0E9A79E
// reference: B9420B48CF052E34, 6FDD6A1C99DF310F
// reference: B9420B488EDBFAA6, BA674B8B6FF1339A
// reference: C022612429CAC9C7, 3E3F62D2D8C36EC0
// reference: BAAADF160C139CB9, 3B8212DC1038224A
// reference: F138ED2A1182AA30, B817BE8F9AF40552
// reference: F51696E7173B3032, D07C0125AD3F01B9
// reference: F716F4084CF04846, 6FDD6A1C99DF310F
// reference: B5372A1237E9B366, 1CB2FE196A34ED92
// reference: 83ABF08ECCF76B42, 5ADC687F067D9F6C
// reference: 2584B8EB081287D7, 22E0D74A8B55120D
// reference: 3F41287A86E18D59, 53598CBDFE71DB11
// reference: F51696E770CCA444, 4B3E71DD2B012BDE
// reference: 2584B8EB381FA280, 1AFB8DE7813BC51D
// reference: A55C6948CCF76B42, 13A54CCD8AA1DDA2
// reference: F716F4080D2E9CD4, BA674B8B6FF1339A
// reference: B9420B484CF04846, 6FDD6A1C99DF310F
// reference: 36CAF90129CAC9C7, 3E3F62D2D8C36EC0
// shader: 8DD9, D4A7DA63EF663625

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)
layout(points) in;
layout(triangle_strip, max_vertices = 30) out;
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

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms gs_uniforms
layout (std140) uniform gs_config {
    pica_uniforms uniforms;
};
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
bool sub_23();
bool sub_24();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_36();
bool sub_37();
bool sub_38();
bool sub_39();
bool sub_40();
bool sub_41();
bool sub_42();
bool sub_43();
bool sub_44();
bool sub_45();
bool sub_46();
bool sub_47();
bool sub_48();
bool sub_49();
bool sub_50();
bool sub_51();
bool sub_52();
bool sub_53();
bool sub_54();
bool sub_55();
bool sub_25();
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
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    uint jmp_to = 502u;
    while (true) {
        switch (jmp_to) {
        case 502u: {
            if (uniforms.b[15]) {
                sub_1();
            } else {
                sub_2();
            }
            conditional_code.x = uniforms.f[77].xxxx.x > reg_tmp15.zzzz.x;
            conditional_code.y = uniforms.f[77].xxxx.y < reg_tmp15.zzzz.y;
            reg_tmp0 = vs_out_attr0[0].xyww + -reg_tmp13.xyww;
            reg_tmp1 = reg_tmp13.xyww + -reg_tmp9.xyww;
            reg_tmp2 = reg_tmp9.xyww + -vs_out_attr0[0].xyww;
            if (any(conditional_code)) {
                { jmp_to = 593u; break; }
            }
            conditional_code = equal(uniforms.f[76].yy, reg_tmp6.xx);
            if (all(conditional_code)) {
                { jmp_to = 555u; break; }
            }
            conditional_code.x = uniforms.f[76].xxxx.x > reg_tmp15.yyyy.x;
            conditional_code.y = uniforms.f[76].xxxx.y < reg_tmp15.yyyy.y;
            reg_tmp3 = mul_s(reg_tmp0.yzxx, reg_tmp1.zxyy);
            reg_tmp4 = mul_s(reg_tmp1.yzxx, reg_tmp2.zxyy);
            reg_tmp5 = mul_s(reg_tmp2.yzxx, reg_tmp0.zxyy);
            if (!conditional_code.x) {
                { jmp_to = 550u; break; }
            }
            reg_tmp3 = fma_s(-reg_tmp0.zxyy, reg_tmp1.yzxx, reg_tmp3);
            reg_tmp4 = fma_s(-reg_tmp1.zxyy, reg_tmp2.yzxx, reg_tmp4);
            reg_tmp5 = fma_s(-reg_tmp2.zxyy, reg_tmp0.yzxx, reg_tmp5);
            reg_tmp0.x = dot_3(reg_tmp3.xyz, reg_tmp13.xyw);
            reg_tmp0.y = dot_3(reg_tmp4.xyz, reg_tmp9.xyw);
            reg_tmp0.z = dot_3(reg_tmp5.xyz, vs_out_attr0[0].xyw);
            conditional_code = equal(reg_tmp11.xy, vs_out_attr0[0].xy);
            if (all(conditional_code)) {
                sub_3();
            }
            reg_tmp0.x = dot_3(reg_tmp0.xyz, reg_tmp15.www);
            reg_tmp0.y = (reg_tmp15.yyyy).y;
            if (all(conditional_code)) {
                sub_4();
                return true;
            } else {
                sub_24();
                return true;
            }
        }
        case 550u: {
            reg_tmp15.w = (-reg_tmp15.wwww).w;
            reg_tmp15.z = (uniforms.f[77].yyyy).z;
            return true;
        }
        case 555u: {
            conditional_code = greaterThan(uniforms.f[76].xx, reg_tmp15.yy);
            reg_tmp15.z = (uniforms.f[76].xxxx).z;
            reg_tmp0 = vs_out_attr0[0].xyww + -reg_tmp13.xyww;
            reg_tmp1 = reg_tmp13.xyww + -reg_tmp11.xyww;
            if (conditional_code.x) {
                sub_26();
                return true;
            }
            return true;
        }
        case 593u: {
            conditional_code.x = uniforms.f[76].zzzz.x > reg_tmp15.zzzz.x;
            conditional_code.y = uniforms.f[76].zzzz.y == reg_tmp15.zzzz.y;
            reg_tmp0 = vs_out_attr0[0].xyww + -reg_tmp13.xyww;
            reg_tmp1 = reg_tmp13.xyww + -reg_tmp11.xyww;
            reg_tmp2 = reg_tmp11.xyww + -vs_out_attr0[0].xyww;
            if (any(conditional_code)) {
                { jmp_to = 666u; break; }
            }
            conditional_code = equal(reg_tmp13.xy, vs_out_attr0[0].xy);
            reg_tmp3 = mul_s(reg_tmp0.yzxx, reg_tmp1.zxyy);
            reg_tmp4 = mul_s(reg_tmp1.yzxx, reg_tmp2.zxyy);
            reg_tmp5 = mul_s(reg_tmp2.yzxx, reg_tmp0.zxyy);
            if (all(conditional_code)) {
                sub_32();
            }
            if (all(conditional_code)) {
                { jmp_to = 662u; break; }
            }
            conditional_code = equal(reg_tmp9.xy, vs_out_attr0[0].xy);
            reg_tmp3 = fma_s(-reg_tmp0.zxyy, reg_tmp1.yzxx, reg_tmp3);
            reg_tmp4 = fma_s(-reg_tmp1.zxyy, reg_tmp2.yzxx, reg_tmp4);
            reg_tmp5 = fma_s(-reg_tmp2.zxyy, reg_tmp0.yzxx, reg_tmp5);
            if (all(conditional_code)) {
                sub_33();
            }
            reg_tmp1 = reg_tmp15.zzzz;
            reg_tmp0.x = dot_3(reg_tmp3.xyz, reg_tmp13.xyw);
            reg_tmp0.y = dot_3(reg_tmp4.xyz, reg_tmp11.xyw);
            reg_tmp0.z = dot_3(reg_tmp5.xyz, vs_out_attr0[0].xyw);
            if (any(not(conditional_code))) {
                sub_34();
            } else {
                sub_35();
            }
            reg_tmp10 = reg_tmp12;
            reg_tmp9 = reg_tmp11;
            reg_tmp7.zw = (reg_tmp8.xyxy).zw;
            reg_tmp12 = reg_tmp14;
            reg_tmp11 = reg_tmp13;
            reg_tmp8.xy = (reg_tmp8.zwzw).xy;
            reg_tmp0.xy = vec2(dot_3(reg_tmp0.xyz, reg_tmp15.www));
            conditional_code = equal(uniforms.f[76].yw, reg_tmp1.yx);
            reg_tmp15.x = (reg_tmp15.yyyy).x;
            reg_tmp15.y = (reg_tmp0.xxxx).y;
            reg_tmp0.y = (mul_s(reg_tmp0, reg_tmp15.xxxx)).y;
            if (!conditional_code.y) {
                sub_36();
            } else {
                sub_43();
            }
            reg_tmp8.zw = (mul_s(uniforms.f[71].xyxy, vs_out_attr2[0].xyxy)).zw;
            reg_tmp13 = vs_out_attr0[0];
            reg_tmp14 = vs_out_attr1[0];
            if (uniforms.b[1]) {
                sub_48();
            }
            reg_tmp15.z = (uniforms.f[77].xxxx).z;
            return true;
        }
        case 662u: {
            reg_tmp6.x = (uniforms.f[76].yyyy).x;
            reg_tmp15.z = (uniforms.f[77].xxxx).z;
            return true;
        }
        case 666u: {
            if (conditional_code.x) {
                { jmp_to = 684u; break; }
            }
            reg_tmp0 = vs_out_attr0[0].xyww + -reg_tmp11.xyww;
            reg_tmp1 = reg_tmp11.xyww + -reg_tmp13.xyww;
            reg_tmp2 = reg_tmp13.xyww + -vs_out_attr0[0].xyww;
            reg_tmp3 = mul_s(reg_tmp0.yzxx, reg_tmp1.zxyy);
            reg_tmp4 = mul_s(reg_tmp1.yzxx, reg_tmp2.zxyy);
            reg_tmp5 = mul_s(reg_tmp2.yzxx, reg_tmp0.zxyy);
            reg_tmp3 = fma_s(-reg_tmp0.zxyy, reg_tmp1.yzxx, reg_tmp3);
            reg_tmp4 = fma_s(-reg_tmp1.zxyy, reg_tmp2.yzxx, reg_tmp4);
            reg_tmp5 = fma_s(-reg_tmp2.zxyy, reg_tmp0.yzxx, reg_tmp5);
            reg_tmp0.x = dot_3(reg_tmp3.xyz, reg_tmp11.xyw);
            reg_tmp0.y = dot_3(reg_tmp4.xyz, reg_tmp13.xyw);
            reg_tmp0.z = dot_3(reg_tmp5.xyz, vs_out_attr0[0].xyw);
            reg_tmp15.z = (uniforms.f[76].yyyy + reg_tmp15.zzzz).z;
            reg_tmp9 = vs_out_attr0[0];
            reg_tmp15.y = dot_3(reg_tmp0.xyz, reg_tmp15.www);
            return true;
        }
        case 684u: {
            conditional_code.x = uniforms.f[76].yyyy.x == reg_tmp15.zzzz.x;
            conditional_code.y = uniforms.f[76].yyyy.y != reg_tmp15.zzzz.y;
            reg_tmp6.x = (uniforms.f[76].xxxx).x;
            reg_tmp15.z = (uniforms.f[76].yyyy + reg_tmp15.zzzz).z;
            if (conditional_code.x) {
                sub_49();
            } else {
                sub_50();
            }
            reg_tmp8.xy = (reg_tmp8.zwzw).xy;
            reg_tmp8.zw = (mul_s(uniforms.f[71].xyxy, vs_out_attr2[0].xyxy)).zw;
            reg_tmp12 = reg_tmp14;
            reg_tmp14 = vs_out_attr1[0];
            if (all(conditional_code)) {
                sub_53();
            }
            reg_tmp11 = reg_tmp13;
            reg_tmp13 = vs_out_attr0[0];
            if (uniforms.b[1]) {
                sub_54();
            }
            if (all(conditional_code)) {
                sub_55();
            }
            return true;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_1() {
    return false;
}
bool sub_2() {
    reg_tmp15.z = (uniforms.f[76].xxxx).z;
    return false;
}
bool sub_3() {
    conditional_code = equal(reg_tmp11.zw, vs_out_attr0[0].zw);
    return false;
}
bool sub_4() {
    if (uniforms.b[0]) {
        sub_5();
        return true;
    } else {
        sub_23();
        return true;
    }
}
bool sub_5() {
    conditional_code = greaterThan(uniforms.f[76].xx, reg_tmp0.yy);
    reg_tmp15.w = (-reg_tmp15.wwww).w;
    reg_tmp15.z = (uniforms.f[77].yyyy).z;
    if (all(conditional_code)) {
        sub_6();
    }
    return true;
}
bool sub_23() {
    reg_tmp15.w = (-reg_tmp15.wwww).w;
    reg_tmp15.z = (uniforms.f[77].yyyy).z;
    return true;
}
bool sub_24() {
    conditional_code.x = uniforms.f[76].xxxx.x <= reg_tmp0.x;
    conditional_code.y = uniforms.f[76].xxxx.y > reg_tmp0.y;
    reg_tmp15.w = (-reg_tmp15.wwww).w;
    reg_tmp15.z = (uniforms.f[77].yyyy).z;
    if (all(conditional_code)) {
        sub_25();
    }
    return true;
}
bool sub_26() {
    reg_tmp2 = reg_tmp11.xyww + -vs_out_attr0[0].xyww;
    reg_tmp3 = mul_s(reg_tmp0.yzxx, reg_tmp1.zxyy);
    reg_tmp4 = mul_s(reg_tmp1.yzxx, reg_tmp2.zxyy);
    reg_tmp5 = mul_s(reg_tmp2.yzxx, reg_tmp0.zxyy);
    reg_tmp3 = fma_s(-reg_tmp0.zxyy, reg_tmp1.yzxx, reg_tmp3);
    reg_tmp4 = fma_s(-reg_tmp1.zxyy, reg_tmp2.yzxx, reg_tmp4);
    reg_tmp5 = fma_s(-reg_tmp2.zxyy, reg_tmp0.yzxx, reg_tmp5);
    conditional_code = equal(reg_tmp9.xy, vs_out_attr0[0].xy);
    reg_tmp0.x = dot_3(reg_tmp3.xyz, reg_tmp13.xyw);
    reg_tmp0.y = dot_3(reg_tmp4.xyz, reg_tmp11.xyw);
    reg_tmp0.z = dot_3(reg_tmp5.xyz, vs_out_attr0[0].xyw);
    if (all(conditional_code)) {
        sub_27();
    }
    reg_tmp10 = reg_tmp12;
    reg_tmp9 = reg_tmp11;
    reg_tmp7.zw = (reg_tmp8.xyxy).zw;
    reg_tmp0.x = dot_3(reg_tmp0.xyz, reg_tmp15.www);
    if (all(conditional_code)) {
        sub_28();
        return true;
    } else {
        sub_31();
        return true;
    }
}
bool sub_27() {
    conditional_code = equal(reg_tmp9.zw, vs_out_attr0[0].zw);
    return false;
}
bool sub_28() {
    if (uniforms.b[0]) {
        sub_29();
        return true;
    } else {
        sub_30();
        return true;
    }
}
bool sub_29() {
    {
        sub_6();
    }
    return true;
}
bool sub_30() {
    return true;
}
bool sub_31() {
    conditional_code = lessThanEqual(uniforms.f[76].xx, reg_tmp0.xx);
    if (conditional_code.x) {
        sub_25();
    }
    return true;
}
bool sub_32() {
    conditional_code = equal(reg_tmp13.zw, vs_out_attr0[0].zw);
    return false;
}
bool sub_33() {
    conditional_code = equal(reg_tmp9.zw, vs_out_attr0[0].zw);
    return false;
}
bool sub_34() {
    reg_tmp1.y = (uniforms.f[76].xxxx).y;
    return false;
}
bool sub_35() {
    reg_tmp1.y = (uniforms.f[76].yyyy).y;
    return false;
}
bool sub_36() {
    if (conditional_code.x) {
        sub_37();
    } else {
        sub_40();
    }
    return false;
}
bool sub_37() {
    if (uniforms.b[0]) {
        sub_38();
    } else {
        sub_39();
    }
    return false;
}
bool sub_38() {
    conditional_code = greaterThan(uniforms.f[76].xx, reg_tmp15.xx);
    if (all(conditional_code)) {
        sub_6();
    }
    return false;
}
bool sub_39() {
    return false;
}
bool sub_40() {
    if (uniforms.b[0]) {
        sub_41();
    } else {
        sub_42();
    }
    if (all(conditional_code)) {
        sub_25();
    }
    return false;
}
bool sub_41() {
    conditional_code = greaterThanEqual(uniforms.f[76].xx, reg_tmp0.yy);
    return false;
}
bool sub_42() {
    conditional_code = greaterThanEqual(uniforms.f[76].xx, reg_tmp0.yy);
    return false;
}
bool sub_43() {
    if (conditional_code.x) {
        sub_44();
    } else {
        sub_47();
    }
    return false;
}
bool sub_44() {
    if (uniforms.b[0]) {
        sub_45();
    } else {
        sub_46();
    }
    return false;
}
bool sub_45() {
    conditional_code = greaterThan(uniforms.f[76].xx, reg_tmp15.yy);
    if (all(conditional_code)) {
        sub_6();
    }
    return false;
}
bool sub_46() {
    return false;
}
bool sub_47() {
    conditional_code.x = uniforms.f[76].xxxx.x <= reg_tmp15.xyyy.x;
    conditional_code.y = uniforms.f[76].xxxx.y > reg_tmp15.xyyy.y;
    if (all(conditional_code)) {
        sub_25();
    }
    return false;
}
bool sub_48() {
    reg_tmp8.zw = (mul_s(reg_tmp8.zwzw, vs_out_attr0[0].wwww)).zw;
    return false;
}
bool sub_49() {
    conditional_code = equal(reg_tmp13.xy, vs_out_attr0[0].xy);
    return false;
}
bool sub_50() {
    if (uniforms.b[2]) {
        sub_51();
    } else {
        sub_52();
    }
    return false;
}
bool sub_51() {
    reg_tmp15.w = (uniforms.f[76].yyyy).w;
    return false;
}
bool sub_52() {
    reg_tmp15.w = (-uniforms.f[76].yyyy).w;
    return false;
}
bool sub_53() {
    conditional_code = equal(reg_tmp13.zw, vs_out_attr0[0].zw);
    return false;
}
bool sub_54() {
    reg_tmp8.zw = (mul_s(reg_tmp8.zwzw, vs_out_attr0[0].wwww)).zw;
    return false;
}
bool sub_55() {
    reg_tmp15.w = (-reg_tmp15.wwww).w;
    reg_tmp15.z = (uniforms.f[76].yyyy).z;
    return false;
}
bool sub_25() {
    output_buffer.attributes[0] = reg_tmp13;
    output_buffer.attributes[0].xy = (reg_tmp13 + reg_tmp8.zwzw).xy;
    output_buffer.attributes[1] = uniforms.f[72];
    setemit(0u, false, false);
    emit();
    output_buffer.attributes[0] = reg_tmp9;
    output_buffer.attributes[0].xy = (reg_tmp9 + reg_tmp7.zwzw).xy;
    output_buffer.attributes[1] = uniforms.f[72];
    setemit(1u, false, false);
    emit();
    output_buffer.attributes[0] = reg_tmp13;
    output_buffer.attributes[1] = reg_tmp14;
    setemit(2u, true, false);
    emit();
    output_buffer.attributes[0] = reg_tmp9;
    output_buffer.attributes[1] = reg_tmp10;
    setemit(0u, true, false);
    emit();
    return false;
}
bool sub_6() {
    reg_tmp0 = mul_s(reg_tmp13.xyyy, reg_tmp9.wwww);
    reg_tmp1 = mul_s(reg_tmp9.xyyy, reg_tmp13.wwww);
    reg_tmp0 = reg_tmp0 + -reg_tmp1;
    reg_tmp0 = abs(reg_tmp0);
    reg_tmp0.x = (mul_s(uniforms.f[74].zzzz, reg_tmp0)).x;
    conditional_code = greaterThan(reg_tmp0.xx, reg_tmp0.yy);
    output_buffer.attributes[1] = uniforms.f[73];
    setemit(0u, false, false);
    if (uniforms.b[3]) {
        sub_7();
    } else {
        sub_8();
    }
    if (conditional_code.x) {
        sub_9();
    } else {
        sub_16();
    }
    return false;
}
bool sub_7() {
    reg_tmp0 = mul_s(uniforms.f[76].zzzz, reg_tmp13.wwww);
    return false;
}
bool sub_8() {
    reg_tmp0 = uniforms.f[76].zzzz;
    return false;
}
bool sub_9() {
    reg_tmp1 = mul_s(uniforms.f[74].yyyy, reg_tmp13);
    reg_tmp2 = mul_s(uniforms.f[74].yyyy, reg_tmp9);
    if (uniforms.b[3]) {
        sub_10();
    } else {
        sub_11();
    }
    if (uniforms.b[4]) {
        sub_12();
    } else {
        sub_13();
    }
    reg_tmp4 = mul_s(uniforms.f[74].wwww, reg_tmp3);
    output_buffer.attributes[0] = reg_tmp1;
    emit();
    reg_tmp2.y = (reg_tmp2 + -reg_tmp4).y;
    reg_tmp1.y = (-reg_tmp0 + reg_tmp1).y;
    output_buffer.attributes[1] = uniforms.f[73];
    setemit(1u, false, false);
    if (uniforms.b[4]) {
        sub_14();
    } else {
        sub_15();
    }
    output_buffer.attributes[0] = reg_tmp1;
    emit();
    output_buffer.attributes[0] = reg_tmp2;
    setemit(2u, true, false);
    emit();
    reg_tmp2.y = (reg_tmp2 + -reg_tmp3).y;
    output_buffer.attributes[0] = reg_tmp2;
    setemit(0u, true, false);
    emit();
    return false;
}
bool sub_10() {
    reg_tmp1.y = (reg_tmp1 + reg_tmp13.wwww).y;
    reg_tmp2.y = (reg_tmp2 + reg_tmp9.wwww).y;
    reg_tmp3 = mul_s(uniforms.f[76].zzzz, reg_tmp9.wwww);
    return false;
}
bool sub_11() {
    reg_tmp1.y = (uniforms.f[76].yyyy + reg_tmp1).y;
    reg_tmp2.y = (uniforms.f[76].yyyy + reg_tmp2).y;
    reg_tmp3 = uniforms.f[76].zzzz;
    return false;
}
bool sub_12() {
    reg_tmp1.z = (fma_s(reg_tmp1.wwww, -uniforms.f[71].zzzz, reg_tmp1)).z;
    return false;
}
bool sub_13() {
    reg_tmp1.z = (-uniforms.f[71].zzzz + reg_tmp1).z;
    return false;
}
bool sub_14() {
    reg_tmp2.z = (fma_s(reg_tmp2.wwww, -uniforms.f[71].zzzz, reg_tmp2)).z;
    return false;
}
bool sub_15() {
    reg_tmp2.z = (-uniforms.f[71].zzzz + reg_tmp2).z;
    return false;
}
bool sub_16() {
    reg_tmp1 = mul_s(uniforms.f[74].xxxx, reg_tmp13);
    reg_tmp2 = mul_s(uniforms.f[74].xxxx, reg_tmp9);
    if (uniforms.b[3]) {
        sub_17();
    } else {
        sub_18();
    }
    if (uniforms.b[4]) {
        sub_19();
    } else {
        sub_20();
    }
    reg_tmp4 = mul_s(uniforms.f[74].wwww, reg_tmp3);
    output_buffer.attributes[0] = reg_tmp1;
    emit();
    reg_tmp2.x = (reg_tmp2 + -reg_tmp4).x;
    reg_tmp1.x = (-reg_tmp0 + reg_tmp1).x;
    output_buffer.attributes[1] = uniforms.f[73];
    setemit(1u, false, false);
    if (uniforms.b[4]) {
        sub_21();
    } else {
        sub_22();
    }
    output_buffer.attributes[0] = reg_tmp1;
    emit();
    output_buffer.attributes[0] = reg_tmp2;
    setemit(2u, true, false);
    emit();
    reg_tmp2.x = (reg_tmp2 + -reg_tmp3).x;
    output_buffer.attributes[0] = reg_tmp2;
    setemit(0u, true, false);
    emit();
    return false;
}
bool sub_17() {
    reg_tmp1.x = (reg_tmp1 + reg_tmp13.wwww).x;
    reg_tmp2.x = (reg_tmp2 + reg_tmp9.wwww).x;
    reg_tmp3 = mul_s(uniforms.f[76].zzzz, reg_tmp9.wwww);
    return false;
}
bool sub_18() {
    reg_tmp1.x = (uniforms.f[76].yyyy + reg_tmp1).x;
    reg_tmp2.x = (uniforms.f[76].yyyy + reg_tmp2).x;
    reg_tmp3 = uniforms.f[76].zzzz;
    return false;
}
bool sub_19() {
    reg_tmp1.z = (fma_s(reg_tmp1.wwww, -uniforms.f[71].zzzz, reg_tmp1)).z;
    return false;
}
bool sub_20() {
    reg_tmp1.z = (-uniforms.f[71].zzzz + reg_tmp1).z;
    return false;
}
bool sub_21() {
    reg_tmp2.z = (fma_s(reg_tmp2.wwww, -uniforms.f[71].zzzz, reg_tmp2)).z;
    return false;
}
bool sub_22() {
    reg_tmp2.z = (-uniforms.f[71].zzzz + reg_tmp2).z;
    return false;
}
// reference: 70004607994EF9F2, D4A7DA63EF663625
// shader: 8B31, 1B84C15A8C4CA262

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
layout(location = 7) in vec4 vs_in_reg7;
layout(location = 8) in vec4 vs_in_reg8;

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

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp4.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.xyyy)).xy;
    address_registers.xy = ivec2(reg_tmp4.xy);
    reg_tmp2.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    reg_tmp2.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    reg_tmp2.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    reg_tmp3.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp3.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp3.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp2 = mul_s(reg_tmp2, vs_in_reg7.xxxx);
    reg_tmp3 = mul_s(reg_tmp3, vs_in_reg7.xxxx);
    reg_tmp4.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    reg_tmp4.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    reg_tmp4.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    reg_tmp2 = fma_s(reg_tmp4, vs_in_reg7.yyyy, reg_tmp2);
    reg_tmp4.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp4.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp4.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp3 = fma_s(reg_tmp4, vs_in_reg7.yyyy, reg_tmp3);
    reg_tmp4.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.zwww)).xy;
    address_registers.xy = ivec2(reg_tmp4.xy);
    reg_tmp4.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    reg_tmp4.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    reg_tmp4.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    reg_tmp2 = fma_s(reg_tmp4, vs_in_reg7.zzzz, reg_tmp2);
    reg_tmp4.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp4.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp4.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    reg_tmp3 = fma_s(reg_tmp4, vs_in_reg7.zzzz, reg_tmp3);
    reg_tmp4.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    reg_tmp4.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    reg_tmp4.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    reg_tmp7 = fma_s(reg_tmp4, vs_in_reg7.wwww, reg_tmp2);
    reg_tmp4.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp4.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp4.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    reg_tmp2 = fma_s(reg_tmp4, vs_in_reg7.wwww, reg_tmp3);
    reg_tmp7.w = (uniforms.f[91].yyyy).w;
    reg_tmp2.w = (uniforms.f[91].xxxx).w;
    reg_tmp8.x = dot_s(reg_tmp2, reg_tmp2);
    reg_tmp8.x = rsq_s(reg_tmp8.x);
    reg_tmp8 = mul_s(reg_tmp2, reg_tmp8.xxxx);
    reg_tmp2.x = dot_3(uniforms.f[7].xyz, reg_tmp8.xyz);
    reg_tmp2.y = dot_3(uniforms.f[8].xyz, reg_tmp8.xyz);
    reg_tmp2.z = dot_3(uniforms.f[9].xyz, reg_tmp8.xyz);
    reg_tmp3.x = dot_3(uniforms.f[4].xyz, reg_tmp2.xyz);
    reg_tmp3.y = dot_3(uniforms.f[5].xyz, reg_tmp2.xyz);
    reg_tmp3.z = dot_3(uniforms.f[6].xyz, reg_tmp2.xyz);
    reg_tmp3.w = (uniforms.f[91].xxxx).w;
    reg_tmp1.x = dot_s(reg_tmp3, reg_tmp3);
    reg_tmp1.x = rsq_s(reg_tmp1.x);
    reg_tmp1 = mul_s(reg_tmp3, reg_tmp1.xxxx);
    reg_tmp2.x = dot_s(uniforms.f[7], reg_tmp7);
    reg_tmp2.y = dot_s(uniforms.f[8], reg_tmp7);
    reg_tmp2.z = dot_s(uniforms.f[9], reg_tmp7);
    reg_tmp2.w = (uniforms.f[91].yyyy).w;
    reg_tmp3.x = dot_s(uniforms.f[4], reg_tmp2);
    reg_tmp3.y = dot_s(uniforms.f[5], reg_tmp2);
    reg_tmp3.z = dot_s(uniforms.f[6], reg_tmp2);
    reg_tmp3.w = (uniforms.f[91].yyyy).w;
    reg_tmp2.x = dot_s(uniforms.f[0], reg_tmp3);
    reg_tmp2.y = dot_s(uniforms.f[1], reg_tmp3);
    reg_tmp2.z = dot_s(uniforms.f[2], reg_tmp3);
    reg_tmp2.w = dot_s(uniforms.f[3], reg_tmp3);
    reg_tmp5 = reg_tmp2;
    vs_out_attr0 = reg_tmp5;
    reg_tmp2.xy = (reg_tmp1.xyyy).xy;
    reg_tmp2.zw = (uniforms.f[91].xxxx).zw;
    reg_tmp3.x = dot_s(reg_tmp2, reg_tmp2);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3 = mul_s(reg_tmp2, reg_tmp3.xxxx);
    reg_tmp6 = reg_tmp3;
    reg_tmp3.x = (min(uniforms.f[92].xxxx, reg_tmp5.wwww)).x;
    reg_tmp3.y = (max(uniforms.f[92].yyyy, reg_tmp3.xxxx)).y;
    reg_tmp2 = vec4(rcp_s(reg_tmp3.y));
    vs_out_attr2.xzw = (mul_s(reg_tmp6.yzww, reg_tmp2.yzww)).xzw;
    vs_out_attr2.y = (mul_s(reg_tmp6.xxxx, reg_tmp2.xxxx)).y;
    reg_tmp3 = mul_s(uniforms.f[11], vs_in_reg1);
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp3);
    return true;
}
// reference: C720C9D614A00273, 1B84C15A8C4CA262
// program: 1B84C15A8C4CA262, D4A7DA63EF663625, 0E284A68CC1A6315
// reference: EA776842CDE390F8, 3B8212DC1038224A
// reference: ABB04FB5994EF9F2, D4A7DA63EF663625
// shader: 8B30, CDE7EEF0967BBA97
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (primary_fragment_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7D2ED0838A330A85, CDE7EEF0967BBA97
// program: 3E3F62D2D8C36EC0, 1CB2FE196A34ED92, CDE7EEF0967BBA97

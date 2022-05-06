// shader: 8B31, 5D6CEA02DF4BAE8B

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

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    // 0: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], vs_in_reg0);
    // 1: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], vs_in_reg0);
    // 2: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], vs_in_reg0);
    // 3: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 4: dp4
    reg_tmp6.x = dot_s(uniforms.f[4], reg_tmp5);
    // 5: dp4
    reg_tmp6.y = dot_s(uniforms.f[5], reg_tmp5);
    // 6: dp4
    reg_tmp6.z = dot_s(uniforms.f[6], reg_tmp5);
    // 7: mov
    reg_tmp6.w = (uniforms.f[91].yyyy).w;
    // 8: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], reg_tmp6);
    // 9: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], reg_tmp6);
    // 10: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], reg_tmp6);
    // 11: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], reg_tmp6);
    // 12: mov
    vs_out_attr0 = reg_tmp5;
    // 13: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 14: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 15: mov
    reg_tmp5 = uniforms.f[10].wwww;
    // 16: add
    reg_tmp5.xz = (-uniforms.f[92].zzzz + reg_tmp5.xzzz).xz;
    // 17: mul
    reg_tmp7 = mul_s(uniforms.f[18], reg_tmp5);
    // 18: mov
    reg_tmp5 = vs_in_reg4;
    // 19: add
    reg_tmp5.xy = (reg_tmp5.xyyy + reg_tmp7.xyyy).xy;
    // 20: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], reg_tmp5);
    // 21: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], reg_tmp5);
    // 22: mov
    vs_out_attr2.z = (reg_tmp5.zzzz).z;
    // 23: mov
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    // 24: mov
    reg_tmp5 = vs_in_reg5;
    // 25: add
    reg_tmp5.zw = (reg_tmp5.zwww + reg_tmp7.zwww).zw;
    // 26: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], reg_tmp5);
    // 27: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], reg_tmp5);
    // 28: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 29: end
    return true;
}
// reference: B0DF74119AFF6859, 5D6CEA02DF4BAE8B
// shader: 8DD9, E4F72E6786B1F5DC

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
// reference: 83ABF08ECCF76B42, E4F72E6786B1F5DC
// shader: 8B30, 84D8A2F946C385D9

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
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: D4F4BEF986E18D59, 84D8A2F946C385D9
// program: 5D6CEA02DF4BAE8B, E4F72E6786B1F5DC, 84D8A2F946C385D9
// shader: 8B30, 8909D5118475BAD5

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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: CE312E68081287D7, 8909D5118475BAD5
// program: 5D6CEA02DF4BAE8B, E4F72E6786B1F5DC, 8909D5118475BAD5
// reference: 635CB3E89AFF6859, 5D6CEA02DF4BAE8B
// shader: 8B31, E03E8EEF6BB1234D

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

bool sub_114_4096();

bool exec_shader() {
    sub_114_4096();
    return true;
}

bool sub_114_4096() {
    // 114: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], vs_in_reg0);
    // 115: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], vs_in_reg0);
    // 116: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], vs_in_reg0);
    // 117: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 118: dp4
    reg_tmp6.x = dot_s(uniforms.f[4], reg_tmp5);
    // 119: dp4
    reg_tmp6.y = dot_s(uniforms.f[5], reg_tmp5);
    // 120: dp4
    reg_tmp6.z = dot_s(uniforms.f[6], reg_tmp5);
    // 121: mov
    reg_tmp6.w = (uniforms.f[91].yyyy).w;
    // 122: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], reg_tmp6);
    // 123: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], reg_tmp6);
    // 124: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], reg_tmp6);
    // 125: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], reg_tmp6);
    // 126: mov
    vs_out_attr0 = reg_tmp5;
    // 127: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 128: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 129: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 130: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 131: mov
    vs_out_attr2.zw = (uniforms.f[91].xxxx).zw;
    // 132: mov
    reg_tmp5.xy = (uniforms.f[92].zzzz).xy;
    // 133: add
    reg_tmp5.y = (reg_tmp5.yyyy + vs_in_reg4.zzzz).y;
    // 134: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], reg_tmp5);
    // 135: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], reg_tmp5);
    // 136: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 137: end
    return true;
}
// reference: 635CB3E8BC214BAF, E03E8EEF6BB1234D
// shader: 8DD9, CEF02063E9A18B2F

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
// reference: A55C6948CCF76B42, CEF02063E9A18B2F
// shader: 8B30, 27D0FDF73042CB3E

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (texcolor1.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor0.g) * (texcolor1.g) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((texcolor0.b) * (texcolor1.b) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_3 = byteround(clamp((rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3 * 2.0, alpha_output_3 * 2.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1A8D7BA91182AA30, 27D0FDF73042CB3E
// program: E03E8EEF6BB1234D, CEF02063E9A18B2F, 27D0FDF73042CB3E
// shader: 8B30, 19D378C2226F0E49

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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: CE312E68381FA280, 19D378C2226F0E49
// program: 5D6CEA02DF4BAE8B, E4F72E6786B1F5DC, 19D378C2226F0E49
// shader: 8B31, 8EE39733041671E6

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

bool sub_139_4096();
bool sub_206_208();
bool sub_208_211();

bool exec_shader() {
    sub_139_4096();
    return true;
}

bool sub_139_4096() {
    // 139: mul
    reg_tmp7.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.xyyy)).xy;
    // 140: mova
    address_registers.xy = ivec2(reg_tmp7.xy);
    // 141: dp4
    reg_tmp5.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 142: dp4
    reg_tmp5.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 143: dp4
    reg_tmp5.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 144: dp3
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 145: dp3
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 146: dp3
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 147: mul
    reg_tmp5 = mul_s(reg_tmp5, vs_in_reg7.xxxx);
    // 148: mul
    reg_tmp6 = mul_s(reg_tmp6, vs_in_reg7.xxxx);
    // 149: dp4
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    // 150: dp4
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    // 151: dp4
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    // 152: mad
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.yyyy, reg_tmp5);
    // 153: dp3
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 154: dp3
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 155: dp3
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 156: mad
    reg_tmp6 = fma_s(reg_tmp7, vs_in_reg7.yyyy, reg_tmp6);
    // 157: mul
    reg_tmp7.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.zwww)).xy;
    // 158: mova
    address_registers.xy = ivec2(reg_tmp7.xy);
    // 159: dp4
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 160: dp4
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 161: dp4
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 162: mad
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.zzzz, reg_tmp5);
    // 163: dp3
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 164: dp3
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 165: dp3
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 166: mad
    reg_tmp6 = fma_s(reg_tmp7, vs_in_reg7.zzzz, reg_tmp6);
    // 167: dp4
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    // 168: dp4
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    // 169: dp4
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    // 170: mad
    reg_tmp8 = fma_s(reg_tmp7, vs_in_reg7.wwww, reg_tmp5);
    // 171: dp3
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 172: dp3
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 173: dp3
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 174: mad
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.wwww, reg_tmp6);
    // 175: mov
    reg_tmp8.w = (uniforms.f[91].yyyy).w;
    // 176: mov
    reg_tmp5.w = (uniforms.f[91].xxxx).w;
    // 177: dp4
    reg_tmp9.x = dot_s(reg_tmp5, reg_tmp5);
    // 178: rsq
    reg_tmp9.x = rsq_s(reg_tmp9.x);
    // 179: mul
    reg_tmp9 = mul_s(reg_tmp5, reg_tmp9.xxxx);
    // 180: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], reg_tmp8);
    // 181: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], reg_tmp8);
    // 182: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], reg_tmp8);
    // 183: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 184: dp4
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp5);
    // 185: dp4
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp5);
    // 186: dp4
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp5);
    // 187: mov
    reg_tmp1.w = (-uniforms.f[91].yyyy).w;
    // 188: mov
    vs_out_attr5 = reg_tmp1;
    // 189: dp3
    reg_tmp5.x = dot_3(uniforms.f[7].xyz, reg_tmp9.xyz);
    // 190: dp3
    reg_tmp5.y = dot_3(uniforms.f[8].xyz, reg_tmp9.xyz);
    // 191: dp3
    reg_tmp5.z = dot_3(uniforms.f[9].xyz, reg_tmp9.xyz);
    // 192: dp3
    reg_tmp6.x = dot_3(uniforms.f[4].xyz, reg_tmp5.xyz);
    // 193: dp3
    reg_tmp6.y = dot_3(uniforms.f[5].xyz, reg_tmp5.xyz);
    // 194: dp3
    reg_tmp6.z = dot_3(uniforms.f[6].xyz, reg_tmp5.xyz);
    // 195: mov
    reg_tmp6.w = (uniforms.f[91].xxxx).w;
    // 196: dp4
    reg_tmp2.x = dot_s(reg_tmp6, reg_tmp6);
    // 197: rsq
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    // 198: mul
    reg_tmp2 = mul_s(reg_tmp6, reg_tmp2.xxxx);
    // 199: cmp
    conditional_code = equal(-uniforms.f[91].yy, reg_tmp2.zz);
    // 200: add
    reg_tmp3 = uniforms.f[91].yyyy + reg_tmp2.zzzz;
    // 201: mul
    reg_tmp3 = mul_s(uniforms.f[92].zzzz, reg_tmp3);
    // 202: mov
    vs_out_attr6.w = (uniforms.f[91].xxxx).w;
    // 203: rsq
    reg_tmp3 = vec4(rsq_s(reg_tmp3.x));
    // 204: mul
    reg_tmp4 = mul_s(uniforms.f[92].zzzz, reg_tmp2);
    // 205: ifc
    if (!conditional_code.x) {
        sub_206_208();
    } else {
        sub_208_211();
    }
    // 211: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], -reg_tmp1);
    // 212: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], -reg_tmp1);
    // 213: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], -reg_tmp1);
    // 214: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], -reg_tmp1);
    // 215: mov
    vs_out_attr0 = reg_tmp5;
    // 216: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 217: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 218: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 219: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 220: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 221: mov
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    // 222: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    // 223: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    // 224: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 225: dp4
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    // 226: dp4
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    // 227: mov
    vs_out_attr4.zw = (uniforms.f[91].xxxx).zw;
    // 228: end
    return true;
}
bool sub_206_208() {
    // 206: rcp
    vs_out_attr6.z = rcp_s(reg_tmp3.x);
    // 207: mul
    vs_out_attr6.xy = (mul_s(reg_tmp4, reg_tmp3)).xy;
    return false;
}
bool sub_208_211() {
    // 208: mov
    vs_out_attr6.x = (uniforms.f[91].yyyy).x;
    // 209: mov
    vs_out_attr6.y = (uniforms.f[91].xxxx).y;
    // 210: mov
    vs_out_attr6.z = (uniforms.f[91].xxxx).z;
    return false;
}
// reference: 546826C4F361ED6F, 8EE39733041671E6
// shader: 8DD9, 30364207046027A3

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
// reference: B5372A1237E9B366, 30364207046027A3
// shader: 8B30, 718F1B5C2F0A1E7F

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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: CE312E688352F8A2, 718F1B5C2F0A1E7F
// program: 8EE39733041671E6, 30364207046027A3, 718F1B5C2F0A1E7F
// shader: 8B31, 0833BBE83416832A

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

bool sub_31_4096();
bool sub_57_59();
bool sub_59_62();

bool exec_shader() {
    sub_31_4096();
    return true;
}

bool sub_31_4096() {
    // 31: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], vs_in_reg0);
    // 32: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], vs_in_reg0);
    // 33: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], vs_in_reg0);
    // 34: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 35: dp4
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp5);
    // 36: dp4
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp5);
    // 37: dp4
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp5);
    // 38: mov
    reg_tmp1.w = (-uniforms.f[91].yyyy).w;
    // 39: mov
    vs_out_attr5 = reg_tmp1;
    // 40: dp3
    reg_tmp5.x = dot_3(uniforms.f[7].xyz, vs_in_reg2.xyz);
    // 41: dp3
    reg_tmp5.y = dot_3(uniforms.f[8].xyz, vs_in_reg2.xyz);
    // 42: dp3
    reg_tmp5.z = dot_3(uniforms.f[9].xyz, vs_in_reg2.xyz);
    // 43: dp3
    reg_tmp6.x = dot_3(uniforms.f[4].xyz, reg_tmp5.xyz);
    // 44: dp3
    reg_tmp6.y = dot_3(uniforms.f[5].xyz, reg_tmp5.xyz);
    // 45: dp3
    reg_tmp6.z = dot_3(uniforms.f[6].xyz, reg_tmp5.xyz);
    // 46: mov
    reg_tmp6.w = (uniforms.f[91].xxxx).w;
    // 47: dp4
    reg_tmp2.x = dot_s(reg_tmp6, reg_tmp6);
    // 48: rsq
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    // 49: mul
    reg_tmp2 = mul_s(reg_tmp6, reg_tmp2.xxxx);
    // 50: cmp
    conditional_code = equal(-uniforms.f[91].yy, reg_tmp2.zz);
    // 51: add
    reg_tmp3 = uniforms.f[91].yyyy + reg_tmp2.zzzz;
    // 52: mul
    reg_tmp3 = mul_s(uniforms.f[92].zzzz, reg_tmp3);
    // 53: mov
    vs_out_attr6.w = (uniforms.f[91].xxxx).w;
    // 54: rsq
    reg_tmp3 = vec4(rsq_s(reg_tmp3.x));
    // 55: mul
    reg_tmp4 = mul_s(uniforms.f[92].zzzz, reg_tmp2);
    // 56: ifc
    if (!conditional_code.x) {
        sub_57_59();
    } else {
        sub_59_62();
    }
    // 62: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], -reg_tmp1);
    // 63: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], -reg_tmp1);
    // 64: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], -reg_tmp1);
    // 65: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], -reg_tmp1);
    // 66: mov
    vs_out_attr0 = reg_tmp5;
    // 67: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 68: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 69: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 70: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 71: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 72: mov
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    // 73: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    // 74: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    // 75: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 76: dp4
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    // 77: dp4
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    // 78: mov
    vs_out_attr4.zw = (uniforms.f[91].xxxx).zw;
    // 79: end
    return true;
}
bool sub_57_59() {
    // 57: rcp
    vs_out_attr6.z = rcp_s(reg_tmp3.x);
    // 58: mul
    vs_out_attr6.xy = (mul_s(reg_tmp4, reg_tmp3)).xy;
    return false;
}
bool sub_59_62() {
    // 59: mov
    vs_out_attr6.x = (uniforms.f[91].yyyy).x;
    // 60: mov
    vs_out_attr6.y = (uniforms.f[91].xxxx).y;
    // 61: mov
    vs_out_attr6.z = (uniforms.f[91].xxxx).z;
    return false;
}
// reference: 546826C41DFBABBA, 0833BBE83416832A
// shader: 8B30, 926D49E7526577DE

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
float alpha_output_0 = byteround(clamp((texcolor0.a) * (texcolor1.g), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: 58589746B4E2FD75, 926D49E7526577DE
// program: 0833BBE83416832A, 30364207046027A3, 926D49E7526577DE
// reference: 87EBE13DF361ED6F, 8EE39733041671E6
// shader: 8B31, 67E600D706F3251E

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
layout(location = 9) in vec4 vs_in_reg9;

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

bool sub_290_4096();
bool sub_357_359();
bool sub_359_362();

bool exec_shader() {
    sub_290_4096();
    return true;
}

bool sub_290_4096() {
    // 290: mul
    reg_tmp6.xy = (mul_s(uniforms.f[92].wwww, vs_in_reg8.xyyy)).xy;
    // 291: mova
    address_registers.xy = ivec2(reg_tmp6.xy);
    // 292: dp4
    reg_tmp4.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 293: dp4
    reg_tmp4.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 294: dp4
    reg_tmp4.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 295: dp3
    reg_tmp5.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 296: dp3
    reg_tmp5.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 297: dp3
    reg_tmp5.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 298: mul
    reg_tmp4 = mul_s(reg_tmp4, vs_in_reg7.xxxx);
    // 299: mul
    reg_tmp5 = mul_s(reg_tmp5, vs_in_reg7.xxxx);
    // 300: dp4
    reg_tmp6.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    // 301: dp4
    reg_tmp6.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    // 302: dp4
    reg_tmp6.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    // 303: mad
    reg_tmp4 = fma_s(reg_tmp6, vs_in_reg7.yyyy, reg_tmp4);
    // 304: dp3
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 305: dp3
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 306: dp3
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 307: mad
    reg_tmp5 = fma_s(reg_tmp6, vs_in_reg7.yyyy, reg_tmp5);
    // 308: mul
    reg_tmp6.xy = (mul_s(uniforms.f[92].wwww, vs_in_reg8.zwww)).xy;
    // 309: mova
    address_registers.xy = ivec2(reg_tmp6.xy);
    // 310: dp4
    reg_tmp6.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 311: dp4
    reg_tmp6.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 312: dp4
    reg_tmp6.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 313: mad
    reg_tmp4 = fma_s(reg_tmp6, vs_in_reg7.zzzz, reg_tmp4);
    // 314: dp3
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 315: dp3
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 316: dp3
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 317: mad
    reg_tmp5 = fma_s(reg_tmp6, vs_in_reg7.zzzz, reg_tmp5);
    // 318: dp4
    reg_tmp6.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    // 319: dp4
    reg_tmp6.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    // 320: dp4
    reg_tmp6.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    // 321: mad
    reg_tmp9 = fma_s(reg_tmp6, vs_in_reg7.wwww, reg_tmp4);
    // 322: dp3
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 323: dp3
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 324: dp3
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 325: mad
    reg_tmp4 = fma_s(reg_tmp6, vs_in_reg7.wwww, reg_tmp5);
    // 326: mov
    reg_tmp9.w = (uniforms.f[92].yyyy).w;
    // 327: mov
    reg_tmp4.w = (uniforms.f[92].xxxx).w;
    // 328: dp4
    reg_tmp10.x = dot_s(reg_tmp4, reg_tmp4);
    // 329: rsq
    reg_tmp10.x = rsq_s(reg_tmp10.x);
    // 330: mul
    reg_tmp10 = mul_s(reg_tmp4, reg_tmp10.xxxx);
    // 331: dp4
    reg_tmp4.x = dot_s(uniforms.f[7], reg_tmp9);
    // 332: dp4
    reg_tmp4.y = dot_s(uniforms.f[8], reg_tmp9);
    // 333: dp4
    reg_tmp4.z = dot_s(uniforms.f[9], reg_tmp9);
    // 334: mov
    reg_tmp4.w = (uniforms.f[92].yyyy).w;
    // 335: dp4
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp4);
    // 336: dp4
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp4);
    // 337: dp4
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp4);
    // 338: mov
    reg_tmp1.w = (-uniforms.f[92].yyyy).w;
    // 339: mov
    vs_out_attr5 = reg_tmp1;
    // 340: dp3
    reg_tmp4.x = dot_3(uniforms.f[7].xyz, reg_tmp10.xyz);
    // 341: dp3
    reg_tmp4.y = dot_3(uniforms.f[8].xyz, reg_tmp10.xyz);
    // 342: dp3
    reg_tmp4.z = dot_3(uniforms.f[9].xyz, reg_tmp10.xyz);
    // 343: dp3
    reg_tmp5.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 344: dp3
    reg_tmp5.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 345: dp3
    reg_tmp5.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 346: mov
    reg_tmp5.w = (uniforms.f[92].xxxx).w;
    // 347: dp4
    reg_tmp2.x = dot_s(reg_tmp5, reg_tmp5);
    // 348: rsq
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    // 349: mul
    reg_tmp2 = mul_s(reg_tmp5, reg_tmp2.xxxx);
    // 350: cmp
    conditional_code = equal(-uniforms.f[92].yy, reg_tmp2.zz);
    // 351: add
    reg_tmp4 = uniforms.f[92].yyyy + reg_tmp2.zzzz;
    // 352: mul
    reg_tmp4 = mul_s(uniforms.f[93].zzzz, reg_tmp4);
    // 353: mov
    vs_out_attr6.w = (uniforms.f[92].xxxx).w;
    // 354: rsq
    reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
    // 355: mul
    reg_tmp5 = mul_s(uniforms.f[93].zzzz, reg_tmp2);
    // 356: ifc
    if (!conditional_code.x) {
        sub_357_359();
    } else {
        sub_359_362();
    }
    // 362: add
    reg_tmp7.xyz = (reg_tmp1.xyzz + -reg_tmp2.xyzz).xyz;
    // 363: mov
    reg_tmp7.w = (reg_tmp1.wwww).w;
    // 364: dp4
    reg_tmp4.x = dot_s(uniforms.f[0], -reg_tmp7);
    // 365: dp4
    reg_tmp4.y = dot_s(uniforms.f[1], -reg_tmp7);
    // 366: dp4
    reg_tmp4.z = dot_s(uniforms.f[2], -reg_tmp7);
    // 367: dp4
    reg_tmp4.w = dot_s(uniforms.f[3], -reg_tmp7);
    // 368: mov
    reg_tmp3 = reg_tmp4;
    // 369: dp4
    reg_tmp4.x = dot_s(uniforms.f[0], -reg_tmp1);
    // 370: dp4
    reg_tmp4.y = dot_s(uniforms.f[1], -reg_tmp1);
    // 371: dp4
    reg_tmp4.z = dot_s(uniforms.f[2], -reg_tmp1);
    // 372: dp4
    reg_tmp4.w = dot_s(uniforms.f[3], -reg_tmp1);
    // 373: mov
    reg_tmp6 = reg_tmp4;
    // 374: min
    reg_tmp5.x = (min(uniforms.f[90].zzzz, reg_tmp6.wwww)).x;
    // 375: max
    reg_tmp5.y = (max(uniforms.f[90].yyyy, reg_tmp5.xxxx)).y;
    // 376: rcp
    reg_tmp4 = vec4(rcp_s(reg_tmp5.y));
    // 377: mul
    reg_tmp5.x = (mul_s(reg_tmp6.wwww, reg_tmp4.xxxx)).x;
    // 378: add
    reg_tmp7.x = (reg_tmp5.xxxx + -reg_tmp6.wwww).x;
    // 379: mad
    reg_tmp7.x = (fma_s(reg_tmp7, uniforms.f[91].wwww, reg_tmp6.wwww)).x;
    // 380: mov
    reg_tmp5.x = (reg_tmp2.yyyy).x;
    // 381: mov
    reg_tmp5.y = (reg_tmp2.xxxx).y;
    // 382: mul
    reg_tmp4.xy = (mul_s(reg_tmp5.xyyy, reg_tmp7.xxxx)).xy;
    // 383: mov
    reg_tmp5.x = (uniforms.f[90].wwww).x;
    // 384: add
    reg_tmp4.z = (-uniforms.f[92].yyyy + vs_in_reg9.xxxx).z;
    // 385: madi
    reg_tmp4.z = (fma_s(reg_tmp4, reg_tmp5.xxxx, uniforms.f[92].yyyy)).z;
    // 386: mul
    reg_tmp4.w = (mul_s(uniforms.f[91].xxxx, reg_tmp4.zzzz)).w;
    // 387: mul
    reg_tmp5.xy = (mul_s(reg_tmp4.xyyy, reg_tmp4.wwww)).xy;
    // 388: mul
    reg_tmp4.xy = (mul_s(uniforms.f[90].xxxx, reg_tmp5.xyyy)).xy;
    // 389: mul
    reg_tmp5.z = (mul_s(reg_tmp4.xxxx, reg_tmp4.xxxx)).z;
    // 390: mul
    reg_tmp5.w = (mul_s(reg_tmp4.yyyy, reg_tmp4.yyyy)).w;
    // 391: add
    reg_tmp5.x = (reg_tmp5.zzzz + reg_tmp5.wwww).x;
    // 392: rsq
    reg_tmp7 = vec4(rsq_s(reg_tmp5.x));
    // 393: rcp
    reg_tmp5 = vec4(rcp_s(reg_tmp7.x));
    // 394: add
    reg_tmp7 = reg_tmp3 + -reg_tmp6;
    // 395: mad
    reg_tmp7 = fma_s(reg_tmp7, reg_tmp5.wwww, reg_tmp6);
    // 396: mov
    vs_out_attr0 = reg_tmp7;
    // 397: add
    reg_tmp5.x = (uniforms.f[93].yyyy + reg_tmp4.zzzz).x;
    // 398: flr
    reg_tmp5.y = (floor(reg_tmp5.xxxx)).y;
    // 399: mul
    reg_tmp6.x = (mul_s(uniforms.f[91].yyyy, reg_tmp5.yyyy)).x;
    // 400: mul
    reg_tmp5.y = (mul_s(uniforms.f[90].wwww, vs_in_reg9.yyyy)).y;
    // 401: add
    reg_tmp4.y = (uniforms.f[91].zzzz + -reg_tmp6.xxxx).y;
    // 402: mad
    reg_tmp4.y = (fma_s(reg_tmp4, reg_tmp5.yyyy, reg_tmp6.xxxx)).y;
    // 403: mul
    reg_tmp5.x = (mul_s(uniforms.f[90].xxxx, reg_tmp4.yyyy)).x;
    // 404: add
    reg_tmp4.x = (uniforms.f[92].yyyy + -reg_tmp5.xxxx).x;
    // 405: mul
    reg_tmp6.x = (mul_s(vs_in_reg1.xxxx, reg_tmp4.xxxx)).x;
    // 406: mul
    reg_tmp6.y = (mul_s(vs_in_reg1.yyyy, reg_tmp4.xxxx)).y;
    // 407: mul
    reg_tmp6.z = (mul_s(vs_in_reg1.zzzz, reg_tmp4.xxxx)).z;
    // 408: mul
    reg_tmp5.xyz = (mul_s(uniforms.f[11].xyzz, reg_tmp6.xyzz)).xyz;
    // 409: mov
    reg_tmp5.w = (uniforms.f[11].wwww).w;
    // 410: mul
    vs_out_attr1 = mul_s(uniforms.f[93].zzzz, reg_tmp5);
    // 411: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 412: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 413: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 414: mov
    vs_out_attr2.w = (uniforms.f[92].xxxx).w;
    // 415: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    // 416: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    // 417: mov
    vs_out_attr3.zw = (uniforms.f[92].xxxx).zw;
    // 418: dp4
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    // 419: dp4
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    // 420: mov
    vs_out_attr4.zw = (uniforms.f[92].xxxx).zw;
    // 421: end
    return true;
}
bool sub_357_359() {
    // 357: rcp
    vs_out_attr6.z = rcp_s(reg_tmp4.x);
    // 358: mul
    vs_out_attr6.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
    return false;
}
bool sub_359_362() {
    // 359: mov
    vs_out_attr6.x = (uniforms.f[92].yyyy).x;
    // 360: mov
    vs_out_attr6.y = (uniforms.f[92].xxxx).y;
    // 361: mov
    vs_out_attr6.z = (uniforms.f[92].xxxx).z;
    return false;
}
// reference: 546826C4909CF237, 67E600D706F3251E
// shader: 8B30, 87A6D2CBDB40A32D

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
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp(min((primary_fragment_color.a) + (secondary_fragment_color.a), 1.0) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: CF82EF1D4C3B45D8, 87A6D2CBDB40A32D
// program: 67E600D706F3251E, 30364207046027A3, 87A6D2CBDB40A32D
// shader: 8B30, 12FCB0259AB30B19

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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: 1EA3006470CCA444, 12FCB0259AB30B19
// program: 5D6CEA02DF4BAE8B, E4F72E6786B1F5DC, 12FCB0259AB30B19
// reference: 1EA30064173B3032, 718F1B5C2F0A1E7F
// program: 5D6CEA02DF4BAE8B, E4F72E6786B1F5DC, 718F1B5C2F0A1E7F
// program: 8EE39733041671E6, 30364207046027A3, 12FCB0259AB30B19
// shader: 8B31, 556BEC86C6221458

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

bool sub_230_4096();
bool sub_266_268();
bool sub_268_271();

bool exec_shader() {
    sub_230_4096();
    return true;
}

bool sub_230_4096() {
    // 230: mul
    reg_tmp5.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.xyyy)).xy;
    // 231: mova
    address_registers.xy = ivec2(reg_tmp5.xy);
    // 232: dp4
    reg_tmp8.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 233: dp4
    reg_tmp8.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 234: dp4
    reg_tmp8.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 235: dp3
    reg_tmp9.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 236: dp3
    reg_tmp9.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 237: dp3
    reg_tmp9.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 238: mov
    reg_tmp8.w = (uniforms.f[91].yyyy).w;
    // 239: mov
    reg_tmp9.w = (uniforms.f[91].xxxx).w;
    // 240: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], reg_tmp8);
    // 241: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], reg_tmp8);
    // 242: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], reg_tmp8);
    // 243: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 244: dp4
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp5);
    // 245: dp4
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp5);
    // 246: dp4
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp5);
    // 247: mov
    reg_tmp1.w = (-uniforms.f[91].yyyy).w;
    // 248: mov
    vs_out_attr5 = reg_tmp1;
    // 249: dp3
    reg_tmp5.x = dot_3(uniforms.f[7].xyz, reg_tmp9.xyz);
    // 250: dp3
    reg_tmp5.y = dot_3(uniforms.f[8].xyz, reg_tmp9.xyz);
    // 251: dp3
    reg_tmp5.z = dot_3(uniforms.f[9].xyz, reg_tmp9.xyz);
    // 252: dp3
    reg_tmp6.x = dot_3(uniforms.f[4].xyz, reg_tmp5.xyz);
    // 253: dp3
    reg_tmp6.y = dot_3(uniforms.f[5].xyz, reg_tmp5.xyz);
    // 254: dp3
    reg_tmp6.z = dot_3(uniforms.f[6].xyz, reg_tmp5.xyz);
    // 255: mov
    reg_tmp6.w = (uniforms.f[91].xxxx).w;
    // 256: dp4
    reg_tmp2.x = dot_s(reg_tmp6, reg_tmp6);
    // 257: rsq
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    // 258: mul
    reg_tmp2 = mul_s(reg_tmp6, reg_tmp2.xxxx);
    // 259: cmp
    conditional_code = equal(-uniforms.f[91].yy, reg_tmp2.zz);
    // 260: add
    reg_tmp3 = uniforms.f[91].yyyy + reg_tmp2.zzzz;
    // 261: mul
    reg_tmp3 = mul_s(uniforms.f[92].zzzz, reg_tmp3);
    // 262: mov
    vs_out_attr6.w = (uniforms.f[91].xxxx).w;
    // 263: rsq
    reg_tmp3 = vec4(rsq_s(reg_tmp3.x));
    // 264: mul
    reg_tmp4 = mul_s(uniforms.f[92].zzzz, reg_tmp2);
    // 265: ifc
    if (!conditional_code.x) {
        sub_266_268();
    } else {
        sub_268_271();
    }
    // 271: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], -reg_tmp1);
    // 272: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], -reg_tmp1);
    // 273: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], -reg_tmp1);
    // 274: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], -reg_tmp1);
    // 275: mov
    vs_out_attr0 = reg_tmp5;
    // 276: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 277: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 278: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 279: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 280: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 281: mov
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    // 282: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    // 283: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    // 284: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 285: dp4
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    // 286: dp4
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    // 287: mov
    vs_out_attr4.zw = (uniforms.f[91].xxxx).zw;
    // 288: end
    return true;
}
bool sub_266_268() {
    // 266: rcp
    vs_out_attr6.z = rcp_s(reg_tmp3.x);
    // 267: mul
    vs_out_attr6.xy = (mul_s(reg_tmp4, reg_tmp3)).xy;
    return false;
}
bool sub_268_271() {
    // 268: mov
    vs_out_attr6.x = (uniforms.f[91].yyyy).x;
    // 269: mov
    vs_out_attr6.y = (uniforms.f[91].xxxx).y;
    // 270: mov
    vs_out_attr6.z = (uniforms.f[91].xxxx).z;
    return false;
}
// reference: 546826C4A116BFCF, 556BEC86C6221458
// program: 556BEC86C6221458, 30364207046027A3, 12FCB0259AB30B19
// reference: 87EBE13D909CF237, 67E600D706F3251E
// reference: 87EBE13DA116BFCF, 556BEC86C6221458
// shader: 8B30, 8F6B5DA3C03E741D

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: 1EA30064A8BEDD7A, 8F6B5DA3C03E741D
// program: 5D6CEA02DF4BAE8B, E4F72E6786B1F5DC, 8F6B5DA3C03E741D
// shader: 8B31, 57649E6ECE396F42

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

bool sub_428_4096();

bool exec_shader() {
    sub_428_4096();
    return true;
}

bool sub_428_4096() {
    // 428: mova
    address_registers.x = (ivec2(vs_in_reg8.xx)).x;
    // 429: mov
    reg_tmp8.x = (uniforms.f[19 + address_registers.x].wwww).x;
    // 430: mov
    reg_tmp6 = uniforms.f[95];
    // 431: mov
    reg_tmp7 = uniforms.f[94];
    // 432: mul
    reg_tmp5.z = (mul_s(reg_tmp8.xxxx, reg_tmp8.xxxx)).z;
    // 433: mad
    reg_tmp5.xy = (fma_s(reg_tmp5.zzzz, reg_tmp6.xyyy, reg_tmp6.zwww)).xy;
    // 434: mad
    reg_tmp5.xy = (fma_s(reg_tmp5.zzzz, reg_tmp5.xyyy, reg_tmp7.xyyy)).xy;
    // 435: mad
    reg_tmp5.xy = (fma_s(reg_tmp5.zzzz, reg_tmp5.xyyy, reg_tmp7.zwww)).xy;
    // 436: mov
    reg_tmp6 = uniforms.f[93];
    // 437: mad
    reg_tmp5.xy = (fma_s(reg_tmp5.zzzz, reg_tmp5.xyyy, reg_tmp6.xyyy)).xy;
    // 438: mad
    reg_tmp5.xy = (fma_s(reg_tmp5.zzzz, reg_tmp5.xyyy, reg_tmp6.zwww)).xy;
    // 439: mul
    reg_tmp5.y = (mul_s(reg_tmp5.yyyy, reg_tmp8.xxxx)).y;
    // 440: mul
    reg_tmp6.x = (mul_s(vs_in_reg0.xxxx, reg_tmp5.xxxx)).x;
    // 441: mul
    reg_tmp6.y = (mul_s(vs_in_reg0.yyyy, reg_tmp5.yyyy)).y;
    // 442: add
    reg_tmp7.x = (reg_tmp6.xxxx + reg_tmp6.yyyy).x;
    // 443: mul
    reg_tmp6.x = (mul_s(-vs_in_reg0.xxxx, reg_tmp5.yyyy)).x;
    // 444: mul
    reg_tmp6.y = (mul_s(vs_in_reg0.yyyy, reg_tmp5.xxxx)).y;
    // 445: add
    reg_tmp7.y = (reg_tmp6.xxxx + reg_tmp6.yyyy).y;
    // 446: mov
    reg_tmp7.zw = (uniforms.f[91].xxxx).zw;
    // 447: add
    reg_tmp8 = uniforms.f[19 + address_registers.x] + reg_tmp7;
    // 448: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], reg_tmp8);
    // 449: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], reg_tmp8);
    // 450: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], reg_tmp8);
    // 451: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 452: dp4
    reg_tmp6.x = dot_s(uniforms.f[4], reg_tmp5);
    // 453: dp4
    reg_tmp6.y = dot_s(uniforms.f[5], reg_tmp5);
    // 454: dp4
    reg_tmp6.z = dot_s(uniforms.f[6], reg_tmp5);
    // 455: mov
    reg_tmp6.w = (uniforms.f[91].yyyy).w;
    // 456: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], reg_tmp6);
    // 457: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], reg_tmp6);
    // 458: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], reg_tmp6);
    // 459: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], reg_tmp6);
    // 460: mov
    vs_out_attr0 = reg_tmp5;
    // 461: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 462: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 463: mov
    reg_tmp5 = uniforms.f[10].wwww;
    // 464: add
    reg_tmp5.xz = (-uniforms.f[92].zzzz + reg_tmp5.xzzz).xz;
    // 465: mul
    reg_tmp7 = mul_s(uniforms.f[18], reg_tmp5);
    // 466: mov
    reg_tmp5 = vs_in_reg4;
    // 467: add
    reg_tmp5.xy = (reg_tmp5.xyyy + reg_tmp7.xyyy).xy;
    // 468: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], reg_tmp5);
    // 469: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], reg_tmp5);
    // 470: mov
    vs_out_attr2.z = (reg_tmp5.zzzz).z;
    // 471: mov
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    // 472: mov
    reg_tmp5 = vs_in_reg5;
    // 473: add
    reg_tmp5.zw = (reg_tmp5.zwww + reg_tmp7.zwww).zw;
    // 474: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], reg_tmp5);
    // 475: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], reg_tmp5);
    // 476: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 477: end
    return true;
}
// reference: 635CB3E886C7A699, 57649E6ECE396F42
// program: 57649E6ECE396F42, E4F72E6786B1F5DC, 8909D5118475BAD5
// shader: 8B30, C6A5B47014DBA717

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (primary_fragment_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 2.0), vec4(0.0), vec4(1.0));
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
// reference: 969B46008A330A85, C6A5B47014DBA717
// program: 8EE39733041671E6, 30364207046027A3, C6A5B47014DBA717

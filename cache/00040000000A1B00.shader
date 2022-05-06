// shader: 8B31, 7E8223C56B3444F6

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
    reg_tmp7.xy = (uniforms.f[10].wwww).xy;
    // 16: add
    reg_tmp7.x = (-uniforms.f[92].zzzz + reg_tmp7.xxxx).x;
    // 17: mul
    reg_tmp7.xy = (mul_s(uniforms.f[18].xyyy, reg_tmp7.xyyy)).xy;
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
    // 25: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], reg_tmp5);
    // 26: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], reg_tmp5);
    // 27: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 28: end
    return true;
}
// reference: 7B2276B4A04C7118, 7E8223C56B3444F6
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
// program: 7E8223C56B3444F6, E4F72E6786B1F5DC, 84D8A2F946C385D9
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
// program: 7E8223C56B3444F6, E4F72E6786B1F5DC, 8909D5118475BAD5
// reference: A8A1B14DA04C7118, 7E8223C56B3444F6
// shader: 8B31, 034C84BD68DE905F

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

bool sub_80_4096();

bool exec_shader() {
    sub_80_4096();
    return true;
}

bool sub_80_4096() {
    // 80: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], vs_in_reg0);
    // 81: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], vs_in_reg0);
    // 82: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], vs_in_reg0);
    // 83: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 84: dp4
    reg_tmp6.x = dot_s(uniforms.f[4], reg_tmp5);
    // 85: dp4
    reg_tmp6.y = dot_s(uniforms.f[5], reg_tmp5);
    // 86: dp4
    reg_tmp6.z = dot_s(uniforms.f[6], reg_tmp5);
    // 87: mov
    reg_tmp6.w = (uniforms.f[91].yyyy).w;
    // 88: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], reg_tmp6);
    // 89: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], reg_tmp6);
    // 90: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], reg_tmp6);
    // 91: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], reg_tmp6);
    // 92: mov
    vs_out_attr0 = reg_tmp5;
    // 93: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 94: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 95: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 96: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 97: mov
    vs_out_attr2.zw = (uniforms.f[91].xxxx).zw;
    // 98: mov
    reg_tmp5.xy = (uniforms.f[92].zzzz).xy;
    // 99: add
    reg_tmp5.y = (reg_tmp5.yyyy + vs_in_reg4.zzzz).y;
    // 100: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], reg_tmp5);
    // 101: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], reg_tmp5);
    // 102: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 103: end
    return true;
}
// reference: A8A1B14D31D5719C, 034C84BD68DE905F
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
// program: 034C84BD68DE905F, CEF02063E9A18B2F, 27D0FDF73042CB3E
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
// program: 7E8223C56B3444F6, E4F72E6786B1F5DC, 19D378C2226F0E49
// shader: 8B31, B72C8F11A2B92DDA

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

bool sub_30_4096();
bool sub_56_58();
bool sub_58_61();

bool exec_shader() {
    sub_30_4096();
    return true;
}

bool sub_30_4096() {
    // 30: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], vs_in_reg0);
    // 31: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], vs_in_reg0);
    // 32: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], vs_in_reg0);
    // 33: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 34: dp4
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp5);
    // 35: dp4
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp5);
    // 36: dp4
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp5);
    // 37: mov
    reg_tmp1.w = (-uniforms.f[91].yyyy).w;
    // 38: mov
    vs_out_attr5 = reg_tmp1;
    // 39: dp3
    reg_tmp5.x = dot_3(uniforms.f[7].xyz, vs_in_reg2.xyz);
    // 40: dp3
    reg_tmp5.y = dot_3(uniforms.f[8].xyz, vs_in_reg2.xyz);
    // 41: dp3
    reg_tmp5.z = dot_3(uniforms.f[9].xyz, vs_in_reg2.xyz);
    // 42: dp3
    reg_tmp6.x = dot_3(uniforms.f[4].xyz, reg_tmp5.xyz);
    // 43: dp3
    reg_tmp6.y = dot_3(uniforms.f[5].xyz, reg_tmp5.xyz);
    // 44: dp3
    reg_tmp6.z = dot_3(uniforms.f[6].xyz, reg_tmp5.xyz);
    // 45: mov
    reg_tmp6.w = (uniforms.f[91].xxxx).w;
    // 46: dp4
    reg_tmp2.x = dot_s(reg_tmp6, reg_tmp6);
    // 47: rsq
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    // 48: mul
    reg_tmp2 = mul_s(reg_tmp6, reg_tmp2.xxxx);
    // 49: cmp
    conditional_code = equal(-uniforms.f[91].yy, reg_tmp2.zz);
    // 50: add
    reg_tmp3 = uniforms.f[91].yyyy + reg_tmp2.zzzz;
    // 51: mul
    reg_tmp3 = mul_s(uniforms.f[92].zzzz, reg_tmp3);
    // 52: mov
    vs_out_attr6.w = (uniforms.f[91].xxxx).w;
    // 53: rsq
    reg_tmp3 = vec4(rsq_s(reg_tmp3.x));
    // 54: mul
    reg_tmp4 = mul_s(uniforms.f[92].zzzz, reg_tmp2);
    // 55: ifc
    if (!conditional_code.x) {
        sub_56_58();
    } else {
        sub_58_61();
    }
    // 61: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], -reg_tmp1);
    // 62: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], -reg_tmp1);
    // 63: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], -reg_tmp1);
    // 64: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], -reg_tmp1);
    // 65: mov
    vs_out_attr0 = reg_tmp5;
    // 66: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 67: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 68: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 69: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 70: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 71: mov
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    // 72: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    // 73: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    // 74: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 75: dp4
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    // 76: dp4
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    // 77: mov
    vs_out_attr4.zw = (uniforms.f[91].xxxx).zw;
    // 78: end
    return true;
}
bool sub_56_58() {
    // 56: rcp
    vs_out_attr6.z = rcp_s(reg_tmp3.x);
    // 57: mul
    vs_out_attr6.xy = (mul_s(reg_tmp4, reg_tmp3)).xy;
    return false;
}
bool sub_58_61() {
    // 58: mov
    vs_out_attr6.x = (uniforms.f[91].yyyy).x;
    // 59: mov
    vs_out_attr6.y = (uniforms.f[91].xxxx).y;
    // 60: mov
    vs_out_attr6.z = (uniforms.f[91].xxxx).z;
    return false;
}
// reference: 9F952461880C20BC, B72C8F11A2B92DDA
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
// program: B72C8F11A2B92DDA, 30364207046027A3, 926D49E7526577DE
// shader: 8B30, 3EDFFC5431AD1E80

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

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CE312E68D060FEE9, 3EDFFC5431AD1E80
// program: 7E8223C56B3444F6, E4F72E6786B1F5DC, 3EDFFC5431AD1E80
// shader: 8B31, 5EF620A978F98BD1

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

bool sub_105_4096();
bool sub_172_174();
bool sub_174_177();

bool exec_shader() {
    sub_105_4096();
    return true;
}

bool sub_105_4096() {
    // 105: mul
    reg_tmp7.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.xyyy)).xy;
    // 106: mova
    address_registers.xy = ivec2(reg_tmp7.xy);
    // 107: dp4
    reg_tmp5.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 108: dp4
    reg_tmp5.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 109: dp4
    reg_tmp5.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 110: dp3
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 111: dp3
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 112: dp3
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 113: mul
    reg_tmp5 = mul_s(reg_tmp5, vs_in_reg7.xxxx);
    // 114: mul
    reg_tmp6 = mul_s(reg_tmp6, vs_in_reg7.xxxx);
    // 115: dp4
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    // 116: dp4
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    // 117: dp4
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    // 118: mad
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.yyyy, reg_tmp5);
    // 119: dp3
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 120: dp3
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 121: dp3
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 122: mad
    reg_tmp6 = fma_s(reg_tmp7, vs_in_reg7.yyyy, reg_tmp6);
    // 123: mul
    reg_tmp7.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.zwww)).xy;
    // 124: mova
    address_registers.xy = ivec2(reg_tmp7.xy);
    // 125: dp4
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 126: dp4
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 127: dp4
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 128: mad
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.zzzz, reg_tmp5);
    // 129: dp3
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 130: dp3
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 131: dp3
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 132: mad
    reg_tmp6 = fma_s(reg_tmp7, vs_in_reg7.zzzz, reg_tmp6);
    // 133: dp4
    reg_tmp7.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    // 134: dp4
    reg_tmp7.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    // 135: dp4
    reg_tmp7.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    // 136: mad
    reg_tmp8 = fma_s(reg_tmp7, vs_in_reg7.wwww, reg_tmp5);
    // 137: dp3
    reg_tmp7.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 138: dp3
    reg_tmp7.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 139: dp3
    reg_tmp7.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 140: mad
    reg_tmp5 = fma_s(reg_tmp7, vs_in_reg7.wwww, reg_tmp6);
    // 141: mov
    reg_tmp8.w = (uniforms.f[91].yyyy).w;
    // 142: mov
    reg_tmp5.w = (uniforms.f[91].xxxx).w;
    // 143: dp4
    reg_tmp9.x = dot_s(reg_tmp5, reg_tmp5);
    // 144: rsq
    reg_tmp9.x = rsq_s(reg_tmp9.x);
    // 145: mul
    reg_tmp9 = mul_s(reg_tmp5, reg_tmp9.xxxx);
    // 146: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], reg_tmp8);
    // 147: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], reg_tmp8);
    // 148: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], reg_tmp8);
    // 149: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 150: dp4
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp5);
    // 151: dp4
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp5);
    // 152: dp4
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp5);
    // 153: mov
    reg_tmp1.w = (-uniforms.f[91].yyyy).w;
    // 154: mov
    vs_out_attr5 = reg_tmp1;
    // 155: dp3
    reg_tmp5.x = dot_3(uniforms.f[7].xyz, reg_tmp9.xyz);
    // 156: dp3
    reg_tmp5.y = dot_3(uniforms.f[8].xyz, reg_tmp9.xyz);
    // 157: dp3
    reg_tmp5.z = dot_3(uniforms.f[9].xyz, reg_tmp9.xyz);
    // 158: dp3
    reg_tmp6.x = dot_3(uniforms.f[4].xyz, reg_tmp5.xyz);
    // 159: dp3
    reg_tmp6.y = dot_3(uniforms.f[5].xyz, reg_tmp5.xyz);
    // 160: dp3
    reg_tmp6.z = dot_3(uniforms.f[6].xyz, reg_tmp5.xyz);
    // 161: mov
    reg_tmp6.w = (uniforms.f[91].xxxx).w;
    // 162: dp4
    reg_tmp2.x = dot_s(reg_tmp6, reg_tmp6);
    // 163: rsq
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    // 164: mul
    reg_tmp2 = mul_s(reg_tmp6, reg_tmp2.xxxx);
    // 165: cmp
    conditional_code = equal(-uniforms.f[91].yy, reg_tmp2.zz);
    // 166: add
    reg_tmp3 = uniforms.f[91].yyyy + reg_tmp2.zzzz;
    // 167: mul
    reg_tmp3 = mul_s(uniforms.f[92].zzzz, reg_tmp3);
    // 168: mov
    vs_out_attr6.w = (uniforms.f[91].xxxx).w;
    // 169: rsq
    reg_tmp3 = vec4(rsq_s(reg_tmp3.x));
    // 170: mul
    reg_tmp4 = mul_s(uniforms.f[92].zzzz, reg_tmp2);
    // 171: ifc
    if (!conditional_code.x) {
        sub_172_174();
    } else {
        sub_174_177();
    }
    // 177: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], -reg_tmp1);
    // 178: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], -reg_tmp1);
    // 179: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], -reg_tmp1);
    // 180: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], -reg_tmp1);
    // 181: mov
    vs_out_attr0 = reg_tmp5;
    // 182: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 183: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 184: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 185: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 186: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 187: mov
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    // 188: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    // 189: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    // 190: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 191: dp4
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    // 192: dp4
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    // 193: mov
    vs_out_attr4.zw = (uniforms.f[91].xxxx).zw;
    // 194: end
    return true;
}
bool sub_172_174() {
    // 172: rcp
    vs_out_attr6.z = rcp_s(reg_tmp3.x);
    // 173: mul
    vs_out_attr6.xy = (mul_s(reg_tmp4, reg_tmp3)).xy;
    return false;
}
bool sub_174_177() {
    // 174: mov
    vs_out_attr6.x = (uniforms.f[91].yyyy).x;
    // 175: mov
    vs_out_attr6.y = (uniforms.f[91].xxxx).y;
    // 176: mov
    vs_out_attr6.z = (uniforms.f[91].xxxx).z;
    return false;
}
// reference: 4C16E398D117D2D2, 5EF620A978F98BD1
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
// program: 5EF620A978F98BD1, 30364207046027A3, 718F1B5C2F0A1E7F
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
// reference: CE312E68E4A56CD4, 12FCB0259AB30B19
// program: 7E8223C56B3444F6, E4F72E6786B1F5DC, 12FCB0259AB30B19
// program: 7E8223C56B3444F6, E4F72E6786B1F5DC, 718F1B5C2F0A1E7F
// shader: 8B31, 24601DE0B71B49F8

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

bool sub_196_4096();
bool sub_232_234();
bool sub_234_237();

bool exec_shader() {
    sub_196_4096();
    return true;
}

bool sub_196_4096() {
    // 196: mul
    reg_tmp5.xy = (mul_s(uniforms.f[91].wwww, vs_in_reg8.xyyy)).xy;
    // 197: mova
    address_registers.xy = ivec2(reg_tmp5.xy);
    // 198: dp4
    reg_tmp8.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 199: dp4
    reg_tmp8.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 200: dp4
    reg_tmp8.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 201: dp3
    reg_tmp9.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 202: dp3
    reg_tmp9.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 203: dp3
    reg_tmp9.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 204: mov
    reg_tmp8.w = (uniforms.f[91].yyyy).w;
    // 205: mov
    reg_tmp9.w = (uniforms.f[91].xxxx).w;
    // 206: dp4
    reg_tmp5.x = dot_s(uniforms.f[7], reg_tmp8);
    // 207: dp4
    reg_tmp5.y = dot_s(uniforms.f[8], reg_tmp8);
    // 208: dp4
    reg_tmp5.z = dot_s(uniforms.f[9], reg_tmp8);
    // 209: mov
    reg_tmp5.w = (uniforms.f[91].yyyy).w;
    // 210: dp4
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp5);
    // 211: dp4
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp5);
    // 212: dp4
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp5);
    // 213: mov
    reg_tmp1.w = (-uniforms.f[91].yyyy).w;
    // 214: mov
    vs_out_attr5 = reg_tmp1;
    // 215: dp3
    reg_tmp5.x = dot_3(uniforms.f[7].xyz, reg_tmp9.xyz);
    // 216: dp3
    reg_tmp5.y = dot_3(uniforms.f[8].xyz, reg_tmp9.xyz);
    // 217: dp3
    reg_tmp5.z = dot_3(uniforms.f[9].xyz, reg_tmp9.xyz);
    // 218: dp3
    reg_tmp6.x = dot_3(uniforms.f[4].xyz, reg_tmp5.xyz);
    // 219: dp3
    reg_tmp6.y = dot_3(uniforms.f[5].xyz, reg_tmp5.xyz);
    // 220: dp3
    reg_tmp6.z = dot_3(uniforms.f[6].xyz, reg_tmp5.xyz);
    // 221: mov
    reg_tmp6.w = (uniforms.f[91].xxxx).w;
    // 222: dp4
    reg_tmp2.x = dot_s(reg_tmp6, reg_tmp6);
    // 223: rsq
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    // 224: mul
    reg_tmp2 = mul_s(reg_tmp6, reg_tmp2.xxxx);
    // 225: cmp
    conditional_code = equal(-uniforms.f[91].yy, reg_tmp2.zz);
    // 226: add
    reg_tmp3 = uniforms.f[91].yyyy + reg_tmp2.zzzz;
    // 227: mul
    reg_tmp3 = mul_s(uniforms.f[92].zzzz, reg_tmp3);
    // 228: mov
    vs_out_attr6.w = (uniforms.f[91].xxxx).w;
    // 229: rsq
    reg_tmp3 = vec4(rsq_s(reg_tmp3.x));
    // 230: mul
    reg_tmp4 = mul_s(uniforms.f[92].zzzz, reg_tmp2);
    // 231: ifc
    if (!conditional_code.x) {
        sub_232_234();
    } else {
        sub_234_237();
    }
    // 237: dp4
    reg_tmp5.x = dot_s(uniforms.f[0], -reg_tmp1);
    // 238: dp4
    reg_tmp5.y = dot_s(uniforms.f[1], -reg_tmp1);
    // 239: dp4
    reg_tmp5.z = dot_s(uniforms.f[2], -reg_tmp1);
    // 240: dp4
    reg_tmp5.w = dot_s(uniforms.f[3], -reg_tmp1);
    // 241: mov
    vs_out_attr0 = reg_tmp5;
    // 242: mul
    reg_tmp6 = mul_s(uniforms.f[11], vs_in_reg1);
    // 243: mul
    vs_out_attr1 = mul_s(uniforms.f[92].zzzz, reg_tmp6);
    // 244: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 245: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 246: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 247: mov
    vs_out_attr2.w = (uniforms.f[91].xxxx).w;
    // 248: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    // 249: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    // 250: mov
    vs_out_attr3.zw = (uniforms.f[91].xxxx).zw;
    // 251: dp4
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    // 252: dp4
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    // 253: mov
    vs_out_attr4.zw = (uniforms.f[91].xxxx).zw;
    // 254: end
    return true;
}
bool sub_232_234() {
    // 232: rcp
    vs_out_attr6.z = rcp_s(reg_tmp3.x);
    // 233: mul
    vs_out_attr6.xy = (mul_s(reg_tmp4, reg_tmp3)).xy;
    return false;
}
bool sub_234_237() {
    // 234: mov
    vs_out_attr6.x = (uniforms.f[91].yyyy).x;
    // 235: mov
    vs_out_attr6.y = (uniforms.f[91].xxxx).y;
    // 236: mov
    vs_out_attr6.z = (uniforms.f[91].xxxx).z;
    return false;
}
// reference: 9F9524612CE285FC, 24601DE0B71B49F8
// program: 24601DE0B71B49F8, 30364207046027A3, 12FCB0259AB30B19
// reference: 9F952461D117D2D2, 5EF620A978F98BD1
// shader: 8B31, 60E5B9C1D4DCFA92

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

bool sub_256_4096();
bool sub_323_325();
bool sub_325_328();
bool sub_374_390();
bool sub_390_406();

bool exec_shader() {
    sub_256_4096();
    return true;
}

bool sub_256_4096() {
    // 256: mul
    reg_tmp6.xy = (mul_s(uniforms.f[92].wwww, vs_in_reg8.xyyy)).xy;
    // 257: mova
    address_registers.xy = ivec2(reg_tmp6.xy);
    // 258: dp4
    reg_tmp4.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 259: dp4
    reg_tmp4.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 260: dp4
    reg_tmp4.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 261: dp3
    reg_tmp5.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 262: dp3
    reg_tmp5.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 263: dp3
    reg_tmp5.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 264: mul
    reg_tmp4 = mul_s(reg_tmp4, vs_in_reg7.xxxx);
    // 265: mul
    reg_tmp5 = mul_s(reg_tmp5, vs_in_reg7.xxxx);
    // 266: dp4
    reg_tmp6.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    // 267: dp4
    reg_tmp6.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    // 268: dp4
    reg_tmp6.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    // 269: mad
    reg_tmp4 = fma_s(reg_tmp6, vs_in_reg7.yyyy, reg_tmp4);
    // 270: dp3
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 271: dp3
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 272: dp3
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 273: mad
    reg_tmp5 = fma_s(reg_tmp6, vs_in_reg7.yyyy, reg_tmp5);
    // 274: mul
    reg_tmp6.xy = (mul_s(uniforms.f[92].wwww, vs_in_reg8.zwww)).xy;
    // 275: mova
    address_registers.xy = ivec2(reg_tmp6.xy);
    // 276: dp4
    reg_tmp6.x = dot_s(uniforms.f[18 + address_registers.x], vs_in_reg0);
    // 277: dp4
    reg_tmp6.y = dot_s(uniforms.f[19 + address_registers.x], vs_in_reg0);
    // 278: dp4
    reg_tmp6.z = dot_s(uniforms.f[20 + address_registers.x], vs_in_reg0);
    // 279: mad
    reg_tmp4 = fma_s(reg_tmp6, vs_in_reg7.zzzz, reg_tmp4);
    // 280: dp3
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 281: dp3
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 282: dp3
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.x].xyz, vs_in_reg2.xyz);
    // 283: mad
    reg_tmp5 = fma_s(reg_tmp6, vs_in_reg7.zzzz, reg_tmp5);
    // 284: dp4
    reg_tmp6.x = dot_s(uniforms.f[18 + address_registers.y], vs_in_reg0);
    // 285: dp4
    reg_tmp6.y = dot_s(uniforms.f[19 + address_registers.y], vs_in_reg0);
    // 286: dp4
    reg_tmp6.z = dot_s(uniforms.f[20 + address_registers.y], vs_in_reg0);
    // 287: mad
    reg_tmp9 = fma_s(reg_tmp6, vs_in_reg7.wwww, reg_tmp4);
    // 288: dp3
    reg_tmp6.x = dot_3(uniforms.f[18 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 289: dp3
    reg_tmp6.y = dot_3(uniforms.f[19 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 290: dp3
    reg_tmp6.z = dot_3(uniforms.f[20 + address_registers.y].xyz, vs_in_reg2.xyz);
    // 291: mad
    reg_tmp4 = fma_s(reg_tmp6, vs_in_reg7.wwww, reg_tmp5);
    // 292: mov
    reg_tmp9.w = (uniforms.f[92].yyyy).w;
    // 293: mov
    reg_tmp4.w = (uniforms.f[92].xxxx).w;
    // 294: dp4
    reg_tmp10.x = dot_s(reg_tmp4, reg_tmp4);
    // 295: rsq
    reg_tmp10.x = rsq_s(reg_tmp10.x);
    // 296: mul
    reg_tmp10 = mul_s(reg_tmp4, reg_tmp10.xxxx);
    // 297: dp4
    reg_tmp4.x = dot_s(uniforms.f[7], reg_tmp9);
    // 298: dp4
    reg_tmp4.y = dot_s(uniforms.f[8], reg_tmp9);
    // 299: dp4
    reg_tmp4.z = dot_s(uniforms.f[9], reg_tmp9);
    // 300: mov
    reg_tmp4.w = (uniforms.f[92].yyyy).w;
    // 301: dp4
    reg_tmp1.x = dot_s(uniforms.f[4], -reg_tmp4);
    // 302: dp4
    reg_tmp1.y = dot_s(uniforms.f[5], -reg_tmp4);
    // 303: dp4
    reg_tmp1.z = dot_s(uniforms.f[6], -reg_tmp4);
    // 304: mov
    reg_tmp1.w = (-uniforms.f[92].yyyy).w;
    // 305: mov
    vs_out_attr5 = reg_tmp1;
    // 306: dp3
    reg_tmp4.x = dot_3(uniforms.f[7].xyz, reg_tmp10.xyz);
    // 307: dp3
    reg_tmp4.y = dot_3(uniforms.f[8].xyz, reg_tmp10.xyz);
    // 308: dp3
    reg_tmp4.z = dot_3(uniforms.f[9].xyz, reg_tmp10.xyz);
    // 309: dp3
    reg_tmp5.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 310: dp3
    reg_tmp5.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 311: dp3
    reg_tmp5.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 312: mov
    reg_tmp5.w = (uniforms.f[92].xxxx).w;
    // 313: dp4
    reg_tmp2.x = dot_s(reg_tmp5, reg_tmp5);
    // 314: rsq
    reg_tmp2.x = rsq_s(reg_tmp2.x);
    // 315: mul
    reg_tmp2 = mul_s(reg_tmp5, reg_tmp2.xxxx);
    // 316: cmp
    conditional_code = equal(-uniforms.f[92].yy, reg_tmp2.zz);
    // 317: add
    reg_tmp4 = uniforms.f[92].yyyy + reg_tmp2.zzzz;
    // 318: mul
    reg_tmp4 = mul_s(uniforms.f[93].zzzz, reg_tmp4);
    // 319: mov
    vs_out_attr6.w = (uniforms.f[92].xxxx).w;
    // 320: rsq
    reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
    // 321: mul
    reg_tmp5 = mul_s(uniforms.f[93].zzzz, reg_tmp2);
    // 322: ifc
    if (!conditional_code.x) {
        sub_323_325();
    } else {
        sub_325_328();
    }
    // 328: add
    reg_tmp7.xyz = (reg_tmp1.xyzz + -reg_tmp2.xyzz).xyz;
    // 329: mov
    reg_tmp7.w = (reg_tmp1.wwww).w;
    // 330: dp4
    reg_tmp4.x = dot_s(uniforms.f[0], -reg_tmp7);
    // 331: dp4
    reg_tmp4.y = dot_s(uniforms.f[1], -reg_tmp7);
    // 332: dp4
    reg_tmp4.z = dot_s(uniforms.f[2], -reg_tmp7);
    // 333: dp4
    reg_tmp4.w = dot_s(uniforms.f[3], -reg_tmp7);
    // 334: mov
    reg_tmp3 = reg_tmp4;
    // 335: dp4
    reg_tmp4.x = dot_s(uniforms.f[0], -reg_tmp1);
    // 336: dp4
    reg_tmp4.y = dot_s(uniforms.f[1], -reg_tmp1);
    // 337: dp4
    reg_tmp4.z = dot_s(uniforms.f[2], -reg_tmp1);
    // 338: dp4
    reg_tmp4.w = dot_s(uniforms.f[3], -reg_tmp1);
    // 339: mov
    reg_tmp6 = reg_tmp4;
    // 340: min
    reg_tmp5.x = (min(uniforms.f[90].zzzz, reg_tmp6.wwww)).x;
    // 341: max
    reg_tmp5.y = (max(uniforms.f[90].yyyy, reg_tmp5.xxxx)).y;
    // 342: rcp
    reg_tmp4 = vec4(rcp_s(reg_tmp5.y));
    // 343: mul
    reg_tmp5.x = (mul_s(reg_tmp6.wwww, reg_tmp4.xxxx)).x;
    // 344: add
    reg_tmp7.x = (reg_tmp5.xxxx + -reg_tmp6.wwww).x;
    // 345: mad
    reg_tmp7.x = (fma_s(reg_tmp7, uniforms.f[91].wwww, reg_tmp6.wwww)).x;
    // 346: mov
    reg_tmp5.x = (reg_tmp2.yyyy).x;
    // 347: mov
    reg_tmp5.y = (reg_tmp2.xxxx).y;
    // 348: mul
    reg_tmp4.xy = (mul_s(reg_tmp5.xyyy, reg_tmp7.xxxx)).xy;
    // 349: mov
    reg_tmp5.x = (uniforms.f[90].wwww).x;
    // 350: add
    reg_tmp4.z = (-uniforms.f[92].yyyy + vs_in_reg9.xxxx).z;
    // 351: madi
    reg_tmp4.z = (fma_s(reg_tmp4, reg_tmp5.xxxx, uniforms.f[92].yyyy)).z;
    // 352: mul
    reg_tmp4.w = (mul_s(uniforms.f[91].xxxx, reg_tmp4.zzzz)).w;
    // 353: mul
    reg_tmp5.xy = (mul_s(reg_tmp4.xyyy, reg_tmp4.wwww)).xy;
    // 354: mul
    reg_tmp4.xy = (mul_s(uniforms.f[90].xxxx, reg_tmp5.xyyy)).xy;
    // 355: mul
    reg_tmp5.z = (mul_s(reg_tmp4.xxxx, reg_tmp4.xxxx)).z;
    // 356: mul
    reg_tmp5.w = (mul_s(reg_tmp4.yyyy, reg_tmp4.yyyy)).w;
    // 357: add
    reg_tmp5.x = (reg_tmp5.zzzz + reg_tmp5.wwww).x;
    // 358: rsq
    reg_tmp7 = vec4(rsq_s(reg_tmp5.x));
    // 359: rcp
    reg_tmp5 = vec4(rcp_s(reg_tmp7.x));
    // 360: add
    reg_tmp7 = reg_tmp3 + -reg_tmp6;
    // 361: mad
    reg_tmp7 = fma_s(reg_tmp7, reg_tmp5.wwww, reg_tmp6);
    // 362: mov
    vs_out_attr0 = reg_tmp7;
    // 363: add
    reg_tmp5.x = (uniforms.f[93].yyyy + reg_tmp4.zzzz).x;
    // 364: flr
    reg_tmp5.y = (floor(reg_tmp5.xxxx)).y;
    // 365: mul
    reg_tmp6.x = (mul_s(uniforms.f[91].yyyy, reg_tmp5.yyyy)).x;
    // 366: mul
    reg_tmp5.y = (mul_s(uniforms.f[90].wwww, vs_in_reg9.yyyy)).y;
    // 367: add
    reg_tmp4.y = (uniforms.f[91].zzzz + -reg_tmp6.xxxx).y;
    // 368: mad
    reg_tmp4.y = (fma_s(reg_tmp4, reg_tmp5.yyyy, reg_tmp6.xxxx)).y;
    // 369: mul
    reg_tmp5.x = (mul_s(uniforms.f[90].xxxx, reg_tmp4.yyyy)).x;
    // 370: add
    reg_tmp4.x = (uniforms.f[92].yyyy + -reg_tmp5.xxxx).x;
    // 371: mov
    reg_tmp4.y = (uniforms.f[95].xxxx).y;
    // 372: cmp
    conditional_code.x = uniforms.f[92].yyyy.x != reg_tmp4.xyyy.x;
    conditional_code.y = uniforms.f[92].yyyy.y == reg_tmp4.xyyy.y;
    // 373: ifc
    if (all(conditional_code)) {
        sub_374_390();
    } else {
        sub_390_406();
    }
    // 406: end
    return true;
}
bool sub_323_325() {
    // 323: rcp
    vs_out_attr6.z = rcp_s(reg_tmp4.x);
    // 324: mul
    vs_out_attr6.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
    return false;
}
bool sub_325_328() {
    // 325: mov
    vs_out_attr6.x = (uniforms.f[92].yyyy).x;
    // 326: mov
    vs_out_attr6.y = (uniforms.f[92].xxxx).y;
    // 327: mov
    vs_out_attr6.z = (uniforms.f[92].xxxx).z;
    return false;
}
bool sub_374_390() {
    // 374: mov
    reg_tmp6.x = (uniforms.f[94].xxxx).x;
    // 375: mov
    reg_tmp6.y = (uniforms.f[94].yyyy).y;
    // 376: mov
    reg_tmp6.z = (uniforms.f[94].zzzz).z;
    // 377: mov
    reg_tmp6.w = (uniforms.f[94].wwww).w;
    // 378: mul
    reg_tmp5 = mul_s(uniforms.f[11], reg_tmp6);
    // 379: mul
    vs_out_attr1 = mul_s(uniforms.f[93].zzzz, reg_tmp5);
    // 380: mov
    vs_out_attr2.x = (uniforms.f[93].wwww).x;
    // 381: mov
    vs_out_attr2.y = (uniforms.f[93].xxxx).y;
    // 382: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 383: mov
    vs_out_attr2.w = (uniforms.f[92].xxxx).w;
    // 384: mov
    vs_out_attr3.x = (uniforms.f[93].wwww).x;
    // 385: mov
    vs_out_attr3.y = (uniforms.f[93].xxxx).y;
    // 386: mov
    vs_out_attr3.zw = (uniforms.f[92].xxxx).zw;
    // 387: mov
    vs_out_attr4.x = (uniforms.f[93].wwww).x;
    // 388: mov
    vs_out_attr4.y = (uniforms.f[93].xxxx).y;
    // 389: mov
    vs_out_attr4.zw = (uniforms.f[92].xxxx).zw;
    return false;
}
bool sub_390_406() {
    // 390: mul
    reg_tmp6.x = (mul_s(vs_in_reg1.xxxx, reg_tmp4.xxxx)).x;
    // 391: mul
    reg_tmp6.y = (mul_s(vs_in_reg1.yyyy, reg_tmp4.xxxx)).y;
    // 392: mul
    reg_tmp6.z = (mul_s(vs_in_reg1.zzzz, reg_tmp4.xxxx)).z;
    // 393: mov
    reg_tmp6.w = (vs_in_reg1.wwww).w;
    // 394: mul
    reg_tmp5 = mul_s(uniforms.f[11], reg_tmp6);
    // 395: mul
    vs_out_attr1 = mul_s(uniforms.f[93].zzzz, reg_tmp5);
    // 396: dp4
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg4);
    // 397: dp4
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg4);
    // 398: mov
    vs_out_attr2.z = (vs_in_reg4.zzzz).z;
    // 399: mov
    vs_out_attr2.w = (uniforms.f[92].xxxx).w;
    // 400: dp4
    vs_out_attr3.x = dot_s(uniforms.f[14], vs_in_reg5);
    // 401: dp4
    vs_out_attr3.y = dot_s(uniforms.f[15], vs_in_reg5);
    // 402: mov
    vs_out_attr3.zw = (uniforms.f[92].xxxx).zw;
    // 403: dp4
    vs_out_attr4.x = dot_s(uniforms.f[16], vs_in_reg6);
    // 404: dp4
    vs_out_attr4.y = dot_s(uniforms.f[17], vs_in_reg6);
    // 405: mov
    vs_out_attr4.zw = (uniforms.f[92].xxxx).zw;
    return false;
}
// reference: 9F9524611D68C804, 60E5B9C1D4DCFA92
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
// program: 60E5B9C1D4DCFA92, 30364207046027A3, 87A6D2CBDB40A32D
// reference: 1EA3006470CCA444, 12FCB0259AB30B19
// reference: 1EA30064173B3032, 718F1B5C2F0A1E7F
// program: 5EF620A978F98BD1, 30364207046027A3, 12FCB0259AB30B19
// shader: 8B30, 6DAFCDEACA9A6A2C

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
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTSigned(0, dot(light_vector, normal))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.ggg) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2 * 4.0, alpha_output_2 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (vec3(1.0) - last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_4 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp(min((primary_fragment_color.a) + (secondary_fragment_color.a), 1.0) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5 * 2.0, alpha_output_5 * 2.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E19BD92CC4ECC0EE, 6DAFCDEACA9A6A2C
// program: 60E5B9C1D4DCFA92, 30364207046027A3, 6DAFCDEACA9A6A2C

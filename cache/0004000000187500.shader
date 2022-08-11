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
// shader: 8B31, 0B81B9C37BD7FC7C

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

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp14.x = dot_s(uniforms.f[8], vs_in_reg0);
    reg_tmp14.y = dot_s(uniforms.f[9], vs_in_reg0);
    reg_tmp14.z = dot_s(uniforms.f[10], vs_in_reg0);
    reg_tmp14.w = dot_s(uniforms.f[11], vs_in_reg0);
    reg_tmp15.x = dot_s(uniforms.f[4], reg_tmp14);
    reg_tmp15.y = dot_s(uniforms.f[5], reg_tmp14);
    reg_tmp15.z = dot_s(uniforms.f[6], reg_tmp14);
    reg_tmp15.w = dot_s(uniforms.f[7], reg_tmp14);
    vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp15);
    vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp15);
    vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp15);
    vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp15);
    vs_out_attr1 = mul_s(uniforms.f[16], vs_in_reg2);
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg1);
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg1);
    vs_out_attr2.z = dot_s(uniforms.f[14], vs_in_reg1);
    vs_out_attr2.w = dot_s(uniforms.f[15], vs_in_reg1);
    return true;
}
// reference: 8A799E4C13203D8D, 0B81B9C37BD7FC7C
// shader: 8B30, 960CD3C338752203
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
depth /= gl_FragCoord.w;
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

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F0B917EE59B57968, 960CD3C338752203
// program: 0B81B9C37BD7FC7C, 219384019281D7FD, 960CD3C338752203
// shader: 8B30, 433C8E3C35153C2F
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
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EA7C877FF10D3FF1, 433C8E3C35153C2F
// program: 0B81B9C37BD7FC7C, 219384019281D7FD, 433C8E3C35153C2F
// reference: EA7C877FF0CF55C6, 433C8E3C35153C2F
// reference: F0B917EE5877135F, 960CD3C338752203
// shader: 8DD9, 1C4CBC8096EA16CD

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
// reference: 5DAD5699F59B3586, 1C4CBC8096EA16CD
// shader: 8B31, FC7F4467554D34E5

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

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
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
        sub_1();
    }
    vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp3);
    vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp3);
    vs_out_attr0.z = dot_s(uniforms.f[2].wzyx, reg_tmp3);
    vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp3);
    conditional_code = greaterThanEqual(uniforms.f[5].yy, reg_tmp0.ww);
    if (all(conditional_code)) {
        sub_2();
    } else {
        sub_3();
    }
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
    if (all(not(conditional_code))) {
        sub_8();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_9();
    }
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_10();
    }
    if (all(conditional_code)) {
        sub_11();
    }
    reg_tmp8 = uniforms.f[5].xxxx;
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop64 = 0u; loop64 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop64) {
        sub_12();
    }
    vs_out_attr2 = reg_tmp5;
    vs_out_attr3 = reg_tmp6;
    vs_out_attr4 = reg_tmp7;
    return true;
}
bool sub_1() {
    reg_tmp4.x = (uniforms.f[4].wwww).x;
    reg_tmp4.y = (-uniforms.f[4].zzzz + reg_tmp4.zzzz).y;
    reg_tmp4.z = rcp_s(reg_tmp4.z);
    reg_tmp4.z = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).z;
    reg_tmp3.x = (fma_s(reg_tmp4.xxxx, reg_tmp4.zzzz, reg_tmp3.xxxx)).x;
    return false;
}
bool sub_2() {
    vs_out_attr1.xyz = (uniforms.f[5].yyyy).xyz;
    vs_out_attr1.w = (reg_tmp0.wwww).w;
    return false;
}
bool sub_3() {
    address_registers.y = (ivec2(reg_tmp0.ww)).y;
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
    if (all(not(conditional_code))) {
        sub_4();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_5();
    }
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_6();
    }
    if (all(conditional_code)) {
        sub_7();
    }
    return false;
}
bool sub_4() {
    vs_out_attr1 = uniforms.f[32 + address_registers.y].wzyx;
    return false;
}
bool sub_5() {
    vs_out_attr1 = uniforms.f[33 + address_registers.y].wzyx;
    return false;
}
bool sub_6() {
    vs_out_attr1 = uniforms.f[34 + address_registers.y].wzyx;
    return false;
}
bool sub_7() {
    vs_out_attr1 = uniforms.f[35 + address_registers.y].wzyx;
    return false;
}
bool sub_8() {
    reg_tmp5 = uniforms.f[5].xyyy;
    reg_tmp6 = uniforms.f[5].xyyy;
    reg_tmp7 = uniforms.f[5].xyyy;
    return false;
}
bool sub_9() {
    reg_tmp5 = uniforms.f[5].yyyy;
    reg_tmp6 = uniforms.f[5].yyyy;
    reg_tmp7 = uniforms.f[5].yyyy;
    return false;
}
bool sub_10() {
    reg_tmp5 = uniforms.f[5].xxyy;
    reg_tmp6 = uniforms.f[5].xxyy;
    reg_tmp7 = uniforms.f[5].xxyy;
    return false;
}
bool sub_11() {
    reg_tmp5 = uniforms.f[5].yxyy;
    reg_tmp6 = uniforms.f[5].yxyy;
    reg_tmp7 = uniforms.f[5].yxyy;
    return false;
}
bool sub_12() {
    conditional_code = equal(uniforms.f[5].yy, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_13();
    }
    conditional_code = lessThan(uniforms.f[5].ww, reg_tmp8.xy);
    if (all(conditional_code)) {
        sub_18();
    }
    reg_tmp8 = uniforms.f[5].yyyy + reg_tmp8;
    return false;
}
bool sub_13() {
    address_registers.y = (ivec2(reg_tmp0.zz)).y;
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
    if (all(not(conditional_code))) {
        sub_14();
    }
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_15();
    }
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_16();
    }
    if (all(conditional_code)) {
        sub_17();
    }
    return false;
}
bool sub_14() {
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].wzzz).xy;
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].wzzz).xy;
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].wzzz).xy;
    return false;
}
bool sub_15() {
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].yzzz).xy;
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].yzzz).xy;
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].yzzz).xy;
    return false;
}
bool sub_16() {
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].wxxx).xy;
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].wxxx).xy;
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].wxxx).xy;
    return false;
}
bool sub_17() {
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].yxxx).xy;
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].yxxx).xy;
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].yxxx).xy;
    return false;
}
bool sub_18() {
    conditional_code = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_19();
    }
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_20();
    }
    return false;
}
bool sub_19() {
    reg_tmp5.xy = (uniforms.f[67 + address_registers.y].yxxx).xy;
    reg_tmp6.xy = (uniforms.f[68 + address_registers.y].yxxx).xy;
    reg_tmp7.xy = (uniforms.f[69 + address_registers.y].yxxx).xy;
    return false;
}
bool sub_20() {
    reg_tmp5.xy = (uniforms.f[67 + address_registers.y].wzzz).xy;
    reg_tmp6.xy = (uniforms.f[68 + address_registers.y].wzzz).xy;
    reg_tmp7.xy = (uniforms.f[69 + address_registers.y].wzzz).xy;
    return false;
}
// reference: 456A7DE6F31AC733, FC7F4467554D34E5
// shader: 8B30, AB594C89B96822D7
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
depth /= gl_FragCoord.w;
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

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2F0FC610DDE9911F, AB594C89B96822D7
// program: FC7F4467554D34E5, 1C4CBC8096EA16CD, AB594C89B96822D7
// reference: EA7C877FA9A3D847, 433C8E3C35153C2F
// reference: 7E4E0B72C4DBE323, 960CD3C338752203
// reference: 648B9BE3350F283B, 433C8E3C35153C2F
// shader: 8DD9, 46D32F0F4A84B1CD

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
    return vec4(vtx.attributes[4].x, vtx.attributes[4].y, vtx.attributes[4].z, vtx.attributes[4].w);
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
    view = vec3(vtx.attributes[5].x, vtx.attributes[5].y, vtx.attributes[5].z);
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
// reference: D4191D46B1AD7CE1, 46D32F0F4A84B1CD
// shader: 8B31, 7118EB54C4403E26

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

bool sub_0();
bool sub_1();
bool sub_2();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp13 = uniforms.f[92].xxxz;
    reg_tmp14 = uniforms.f[92].xxxz;
    reg_tmp13.xyz = (vs_in_reg0.xyzz).xyz;
    reg_tmp14.x = dot_s(uniforms.f[8], reg_tmp13);
    reg_tmp14.y = dot_s(uniforms.f[9], reg_tmp13);
    reg_tmp14.z = dot_s(uniforms.f[10], reg_tmp13);
    reg_tmp13.x = dot_s(uniforms.f[4], reg_tmp14);
    reg_tmp13.y = dot_s(uniforms.f[5], reg_tmp14);
    reg_tmp13.z = dot_s(uniforms.f[6], reg_tmp14);
    reg_tmp8.x = dot_s(uniforms.f[0], reg_tmp13);
    reg_tmp8.y = dot_s(uniforms.f[1], reg_tmp13);
    reg_tmp8.z = dot_s(uniforms.f[2], reg_tmp13);
    reg_tmp8.w = dot_s(uniforms.f[3], reg_tmp13);
    vs_out_attr0 = reg_tmp8;
    vs_out_attr5 = -reg_tmp13;
    reg_tmp14.x = dot_3(uniforms.f[8].xyz, vs_in_reg3.xyz);
    reg_tmp14.y = dot_3(uniforms.f[9].xyz, vs_in_reg3.xyz);
    reg_tmp14.z = dot_3(uniforms.f[10].xyz, vs_in_reg3.xyz);
    reg_tmp11.x = dot_3(uniforms.f[4].xyz, reg_tmp14.xyz);
    reg_tmp11.y = dot_3(uniforms.f[5].xyz, reg_tmp14.xyz);
    reg_tmp11.z = dot_3(uniforms.f[6].xyz, reg_tmp14.xyz);
    reg_tmp15 = vs_in_reg4;
    reg_tmp15 = reg_tmp15 + -vs_in_reg2;
    reg_tmp15 = fma_s(reg_tmp15, uniforms.f[17], vs_in_reg2);
    vs_out_attr1 = mul_s(uniforms.f[16], reg_tmp15);
    reg_tmp0.x = dot_3(reg_tmp11.xyz, reg_tmp11.xyz);
    reg_tmp0.x = rsq_s(reg_tmp0.x);
    reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp0.xxxx)).xyz;
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg1);
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg1);
    vs_out_attr2.z = dot_s(uniforms.f[14], vs_in_reg1);
    vs_out_attr2.w = dot_s(uniforms.f[15], vs_in_reg1);
    vs_out_attr3.x = dot_s(uniforms.f[12], vs_in_reg1);
    vs_out_attr3.y = dot_s(uniforms.f[13], vs_in_reg1);
    vs_out_attr3.z = dot_s(uniforms.f[14], vs_in_reg1);
    vs_out_attr3.w = dot_s(uniforms.f[15], vs_in_reg1);
    reg_tmp12 = uniforms.f[92].xxxx;
    conditional_code = equal(-uniforms.f[92].zz, reg_tmp11.zz);
    if (!conditional_code.x) {
        sub_1();
    } else {
        sub_2();
    }
    vs_out_attr4 = reg_tmp12;
    return true;
}
bool sub_1() {
    reg_tmp9 = uniforms.f[92].zzzz + reg_tmp11.zzzz;
    reg_tmp9 = mul_s(uniforms.f[92].yyyy, reg_tmp9);
    reg_tmp9 = vec4(rsq_s(reg_tmp9.z));
    reg_tmp10.xy = (mul_s(uniforms.f[92].yyyy, reg_tmp11)).xy;
    reg_tmp12.z = rcp_s(reg_tmp9.z);
    reg_tmp12.xy = (mul_s(reg_tmp10.xyyy, reg_tmp9.xxxx)).xy;
    return false;
}
bool sub_2() {
    reg_tmp12.x = (uniforms.f[92].zzzz).x;
    return false;
}
// reference: D799AE06CBCC4780, 7118EB54C4403E26
// shader: 8B30, 814FC2691D6853AD
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
depth /= gl_FragCoord.w;
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
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[1].position + view);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(17, clamp(light_src[1].dist_atten_scale * length(-view - light_src[1].position) + light_src[1].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((rounded_primary_color.rgb) + (primary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5 * 4.0, alpha_output_5 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E07236FFDF23F4B4, 814FC2691D6853AD
// program: 7118EB54C4403E26, 46D32F0F4A84B1CD, 814FC2691D6853AD
// reference: 9908565B1F12D47E, 960CD3C338752203
// program: 7118EB54C4403E26, 46D32F0F4A84B1CD, 960CD3C338752203
// shader: 8B31, 78A17CA9418EF229

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

bool sub_0();
bool sub_1();
bool sub_2();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp14 = uniforms.f[92].xxxz;
    reg_tmp15 = uniforms.f[92].xxxz;
    reg_tmp14.xyz = (vs_in_reg0.xyzz).xyz;
    reg_tmp15.x = dot_s(uniforms.f[8], reg_tmp14);
    reg_tmp15.y = dot_s(uniforms.f[9], reg_tmp14);
    reg_tmp15.z = dot_s(uniforms.f[10], reg_tmp14);
    reg_tmp14.x = dot_s(uniforms.f[4], reg_tmp15);
    reg_tmp14.y = dot_s(uniforms.f[5], reg_tmp15);
    reg_tmp14.z = dot_s(uniforms.f[6], reg_tmp15);
    reg_tmp9.x = dot_s(uniforms.f[0], reg_tmp14);
    reg_tmp9.y = dot_s(uniforms.f[1], reg_tmp14);
    reg_tmp9.z = dot_s(uniforms.f[2], reg_tmp14);
    reg_tmp9.w = dot_s(uniforms.f[3], reg_tmp14);
    vs_out_attr0 = reg_tmp9;
    vs_out_attr5 = -reg_tmp14;
    reg_tmp15.x = dot_3(uniforms.f[8].xyz, vs_in_reg3.xyz);
    reg_tmp15.y = dot_3(uniforms.f[9].xyz, vs_in_reg3.xyz);
    reg_tmp15.z = dot_3(uniforms.f[10].xyz, vs_in_reg3.xyz);
    reg_tmp12.x = dot_3(uniforms.f[4].xyz, reg_tmp15.xyz);
    reg_tmp12.y = dot_3(uniforms.f[5].xyz, reg_tmp15.xyz);
    reg_tmp12.z = dot_3(uniforms.f[6].xyz, reg_tmp15.xyz);
    vs_out_attr1 = mul_s(uniforms.f[16], vs_in_reg2);
    reg_tmp0.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
    reg_tmp0.x = rsq_s(reg_tmp0.x);
    reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp0.xxxx)).xyz;
    vs_out_attr2.x = dot_s(uniforms.f[12], vs_in_reg1);
    vs_out_attr2.y = dot_s(uniforms.f[13], vs_in_reg1);
    vs_out_attr2.z = dot_s(uniforms.f[14], vs_in_reg1);
    vs_out_attr2.w = dot_s(uniforms.f[15], vs_in_reg1);
    vs_out_attr3.x = dot_s(uniforms.f[12], vs_in_reg1);
    vs_out_attr3.y = dot_s(uniforms.f[13], vs_in_reg1);
    vs_out_attr3.z = dot_s(uniforms.f[14], vs_in_reg1);
    vs_out_attr3.w = dot_s(uniforms.f[15], vs_in_reg1);
    reg_tmp13 = uniforms.f[92].xxxx;
    conditional_code = equal(-uniforms.f[92].zz, reg_tmp12.zz);
    if (!conditional_code.x) {
        sub_1();
    } else {
        sub_2();
    }
    vs_out_attr4 = reg_tmp13;
    return true;
}
bool sub_1() {
    reg_tmp10 = uniforms.f[92].zzzz + reg_tmp12.zzzz;
    reg_tmp10 = mul_s(uniforms.f[92].yyyy, reg_tmp10);
    reg_tmp10 = vec4(rsq_s(reg_tmp10.z));
    reg_tmp11.xy = (mul_s(uniforms.f[92].yyyy, reg_tmp12)).xy;
    reg_tmp13.z = rcp_s(reg_tmp10.z);
    reg_tmp13.xy = (mul_s(reg_tmp11.xyyy, reg_tmp10.xxxx)).xy;
    return false;
}
bool sub_2() {
    reg_tmp13.x = (uniforms.f[92].zzzz).x;
    return false;
}
// reference: F03C76DD35364667, 78A17CA9418EF229
// shader: 8B30, 58C0B871FBD0446F
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
depth /= gl_FragCoord.w;
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
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
geo_factor = dot(half_vector, half_vector);
geo_factor = geo_factor == 0.0 ? 0.0 : min(dot_product / geo_factor, 1.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) * geo_factor) + (((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1) * geo_factor)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((rounded_primary_color.rgb) + (primary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) - (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 19196CF0F5E090D9, 58C0B871FBD0446F
// program: 78A17CA9418EF229, 46D32F0F4A84B1CD, 58C0B871FBD0446F
// reference: 83CDC6CAEEC61F66, 433C8E3C35153C2F
// shader: 8B31, 450B78E5B046606F

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

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp13 = vs_in_reg2;
    reg_tmp13.zw = (uniforms.f[92].zzzz).zw;
    reg_tmp13.xy = (uniforms.f[92].wwww + reg_tmp13.xyyy).xy;
    reg_tmp14.x = dot_s(uniforms.f[8], reg_tmp13);
    reg_tmp14.y = dot_s(uniforms.f[9], reg_tmp13);
    reg_tmp14.z = dot_s(uniforms.f[10], reg_tmp13);
    reg_tmp14.w = dot_s(uniforms.f[11], reg_tmp13);
    reg_tmp15.x = dot_s(uniforms.f[4], reg_tmp14);
    reg_tmp15.y = dot_s(uniforms.f[5], reg_tmp14);
    reg_tmp15.z = dot_s(uniforms.f[6], reg_tmp14);
    reg_tmp15.w = dot_s(uniforms.f[7], reg_tmp14);
    reg_tmp13.x = dot_s(uniforms.f[0], reg_tmp15);
    reg_tmp13.y = dot_s(uniforms.f[1], reg_tmp15);
    reg_tmp13.z = dot_s(uniforms.f[2], reg_tmp15);
    reg_tmp13.w = dot_s(uniforms.f[3], reg_tmp15);
    reg_tmp15 = mul_s(uniforms.f[91], vs_in_reg0);
    reg_tmp12.x = dot_s(uniforms.f[0], reg_tmp15);
    reg_tmp12.y = dot_s(uniforms.f[1], reg_tmp15);
    reg_tmp12.z = dot_s(uniforms.f[2], reg_tmp15);
    reg_tmp12.w = dot_s(uniforms.f[3], reg_tmp15);
    reg_tmp12 = uniforms.f[90] + reg_tmp12;
    reg_tmp13.x = (reg_tmp13.xxxx + -reg_tmp12.xxxx).x;
    reg_tmp13.y = (reg_tmp13.yyyy + reg_tmp12.yyyy).y;
    vs_out_attr0 = reg_tmp13;
    reg_tmp11 = uniforms.f[16];
    reg_tmp11.w = (mul_s(uniforms.f[16].wwww, vs_in_reg2.wwww)).w;
    vs_out_attr1 = reg_tmp11;
    vs_out_attr2.x = dot_s(uniforms.f[12], reg_tmp13);
    vs_out_attr2.y = dot_s(uniforms.f[13], reg_tmp13);
    vs_out_attr2.z = dot_s(uniforms.f[14], reg_tmp13);
    vs_out_attr2.w = dot_s(uniforms.f[15], reg_tmp13);
    return true;
}
// reference: 005F182964D2E377, 450B78E5B046606F
// program: 450B78E5B046606F, 219384019281D7FD, 960CD3C338752203
// shader: 8B30, 440557D916C77CD7
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
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

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

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 35CA56817551D786, 440557D916C77CD7
// program: FC7F4467554D34E5, 1C4CBC8096EA16CD, 440557D916C77CD7

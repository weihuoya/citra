// shader: 8DD9, B6B95AFD9466EC70

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
// reference: 5DAD5699F59B3586, B6B95AFD9466EC70
// shader: 8B31, A150FC784311F5E1

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0.x = dot_s(uniforms.f[0], vs_in_reg0);
reg_tmp0.y = dot_s(uniforms.f[1], vs_in_reg0);
reg_tmp0.z = dot_s(uniforms.f[2], vs_in_reg0);
reg_tmp0.w = (uniforms.f[77].wwww).w;
reg_tmp2.x = dot_s(uniforms.f[8], vs_in_reg2);
reg_tmp2.y = dot_s(uniforms.f[9], vs_in_reg2);
reg_tmp2.z = dot_s(uniforms.f[10], vs_in_reg2);
reg_tmp2.w = (uniforms.f[78].wwww).w;
vs_out_attr0.x = dot_s(uniforms.f[3], reg_tmp0);
vs_out_attr0.y = dot_s(uniforms.f[4], reg_tmp0);
vs_out_attr0.z = dot_s(uniforms.f[5], reg_tmp0);
vs_out_attr0.w = dot_s(uniforms.f[6], reg_tmp0);
vs_out_attr2 = reg_tmp2;
vs_out_attr3 = reg_tmp2;
vs_out_attr4 = reg_tmp2;
vs_out_attr1 = mul_s(uniforms.f[7], vs_in_reg1);
return true;
}
// reference: 129E4FCA7D1E73FB, A150FC784311F5E1
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
// reference: 37A2E279863B523C, 920194F571A1C470
// program: A150FC784311F5E1, B6B95AFD9466EC70, 920194F571A1C470
// shader: 8B30, B4424FD6FB68AEF8
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
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 00310758863B523C, B4424FD6FB68AEF8
// program: A150FC784311F5E1, B6B95AFD9466EC70, B4424FD6FB68AEF8
// shader: 8B30, E44E5648C89E23AC
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
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 65340EFA863B523C, E44E5648C89E23AC
// program: A150FC784311F5E1, B6B95AFD9466EC70, E44E5648C89E23AC
// shader: 8B30, DC654CC8C89E23AC
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
if (last_tex_env_out.a != alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A0599A6C863B523C, DC654CC8C89E23AC
// program: A150FC784311F5E1, B6B95AFD9466EC70, DC654CC8C89E23AC
// shader: 8B30, A35DA0B3C89E23AC
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
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 030DC4B6863B523C, A35DA0B3C89E23AC
// program: A150FC784311F5E1, B6B95AFD9466EC70, A35DA0B3C89E23AC
// shader: 8B30, BD557BD26EEBA435
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
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) + (const_color[2].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (texcolor0.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BFCFDA2792DD2900, BD557BD26EEBA435
// program: A150FC784311F5E1, B6B95AFD9466EC70, BD557BD26EEBA435
// shader: 8B30, B00E92176AA48E3A
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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 591674EB863B523C, B00E92176AA48E3A
// program: A150FC784311F5E1, B6B95AFD9466EC70, B00E92176AA48E3A
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
// reference: 349E2197863B523C, 4A60600E5756925D
// program: A150FC784311F5E1, B6B95AFD9466EC70, 4A60600E5756925D
// shader: 8DD9, C8DF8CC87A4E8CE6

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
// reference: CD8210802464D9AE, C8DF8CC87A4E8CE6
// shader: 8B31, 6B4C12038C932EEC

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
layout(location=8) in vec4 vs_in_reg8;

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

bool sub_21();
bool sub_4();
bool sub_9();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_5();
bool sub_7();
bool sub_8();
bool sub_10();
bool sub_20();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_6();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_36();
bool sub_42();
bool sub_43();
bool sub_44();
bool sub_45();
bool sub_47();
bool sub_37();
bool sub_38();
bool sub_39();
bool sub_40();
bool sub_41();
bool sub_46();
bool sub_48();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_21() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_9() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_24();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_3();
} else {
sub_7();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_23();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_3() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_4();
}
if (uniforms.b[8]) {
sub_5();
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
sub_6();
return false;
}
bool sub_5() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_4();
}
return false;
}
bool sub_7() {
if (all(bool_regs)) {
sub_8();
} else {
sub_20();
}
return false;
}
bool sub_8() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_9();
}
if (uniforms.b[8]) {
sub_10();
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
sub_11();
return false;
}
bool sub_10() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_9();
}
return false;
}
bool sub_20() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_21();
}
if (uniforms.b[8]) {
sub_22();
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
bool sub_22() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_21();
}
return false;
}
bool sub_23() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_24() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_25();
} else {
sub_26();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_27();
} else {
sub_28();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_31();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_25() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_26() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_27() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_6();
return false;
}
bool sub_28() {
if (all(bool_regs)) {
sub_29();
} else {
sub_30();
}
return false;
}
bool sub_29() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_11();
return false;
}
bool sub_30() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_31() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_6() {
uint jmp_to = 218u;
while (true) {
switch (jmp_to) {
case 218u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 234u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 234u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 234u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_11() {
uint jmp_to = 235u;
while (true) {
switch (jmp_to) {
case 235u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 310u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 272u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 310u; break;
}
case 272u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_12();
} else {
sub_17();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 310u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_12() {
if (bool_regs.y) {
sub_13();
} else {
sub_14();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_13() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_14() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_15();
} else {
sub_16();
}
return false;
}
bool sub_15() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_17() {
if (bool_regs.y) {
sub_18();
} else {
sub_19();
}
return false;
}
bool sub_18() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_19() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_32() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9 = uniforms.f[21];
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (bool_regs.y) {
sub_33();
}
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_33() {
if (uniforms.b[7]) {
sub_34();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_34() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_35() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_36();
} else {
sub_42();
}
return false;
}
bool sub_36() {
sub_37();
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_42() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_43();
} else {
sub_44();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_43() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_44() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_45();
} else {
sub_47();
}
return false;
}
bool sub_45() {
sub_46();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_47() {
sub_48();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_37() {
bool_regs = equal(uniforms.f[93].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
sub_38();
} else {
sub_39();
}
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_38() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
return false;
}
bool sub_39() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_40();
} else {
sub_41();
}
return false;
}
bool sub_40() {
reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
return false;
}
bool sub_41() {
reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
return false;
}
bool sub_46() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_48() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_0() {
sub_1();
sub_32();
sub_35();
return true;
}
// reference: E5252DEE023EF85B, 6B4C12038C932EEC
// shader: 8B30, 9919954F6740CFCD
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texture(tex_cube, vec3(texcoord0, texcoord0_w)).rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texture(tex_cube, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
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
// reference: 74D33FD6F76FBB71, 9919954F6740CFCD
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 9919954F6740CFCD
// shader: 8DD9, EADA2116091C5E01

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
// reference: FC74FA4ACA1C8C74, EADA2116091C5E01
// shader: 8B31, BDB4CD9AD6AFB6D9

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
bool sub_21();
bool sub_4();
bool sub_9();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_5();
bool sub_7();
bool sub_8();
bool sub_10();
bool sub_20();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_6();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_36();
bool sub_42();
bool sub_43();
bool sub_44();
bool sub_45();
bool sub_47();
bool sub_37();
bool sub_38();
bool sub_39();
bool sub_40();
bool sub_41();
bool sub_46();
bool sub_48();
bool sub_49();
bool sub_50();
bool sub_51();
bool sub_52();
bool sub_53();
bool sub_54();
bool sub_55();
bool sub_56();
bool sub_57();
bool sub_58();
bool sub_59();
bool sub_60();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
sub_1();
sub_32();
sub_35();
sub_49();
sub_56();
return true;
}
bool sub_21() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_9() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_24();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_3();
} else {
sub_7();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_23();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_3() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_4();
}
if (uniforms.b[8]) {
sub_5();
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
sub_6();
return false;
}
bool sub_5() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_4();
}
return false;
}
bool sub_7() {
if (all(bool_regs)) {
sub_8();
} else {
sub_20();
}
return false;
}
bool sub_8() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_9();
}
if (uniforms.b[8]) {
sub_10();
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
sub_11();
return false;
}
bool sub_10() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_9();
}
return false;
}
bool sub_20() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_21();
}
if (uniforms.b[8]) {
sub_22();
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
bool sub_22() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_21();
}
return false;
}
bool sub_23() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_24() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_25();
} else {
sub_26();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_27();
} else {
sub_28();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_31();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_25() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_26() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_27() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_6();
return false;
}
bool sub_28() {
if (all(bool_regs)) {
sub_29();
} else {
sub_30();
}
return false;
}
bool sub_29() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_11();
return false;
}
bool sub_30() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_31() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_6() {
uint jmp_to = 218u;
while (true) {
switch (jmp_to) {
case 218u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 234u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 234u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 234u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_11() {
uint jmp_to = 235u;
while (true) {
switch (jmp_to) {
case 235u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 310u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 272u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 310u; break;
}
case 272u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_12();
} else {
sub_17();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 310u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_12() {
if (bool_regs.y) {
sub_13();
} else {
sub_14();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_13() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_14() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_15();
} else {
sub_16();
}
return false;
}
bool sub_15() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_17() {
if (bool_regs.y) {
sub_18();
} else {
sub_19();
}
return false;
}
bool sub_18() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_19() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_32() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9 = uniforms.f[21];
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (bool_regs.y) {
sub_33();
}
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_33() {
if (uniforms.b[7]) {
sub_34();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_34() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_35() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_36();
} else {
sub_42();
}
return false;
}
bool sub_36() {
sub_37();
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_42() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_43();
} else {
sub_44();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_43() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_44() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_45();
} else {
sub_47();
}
return false;
}
bool sub_45() {
sub_46();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_47() {
sub_48();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_37() {
bool_regs = equal(uniforms.f[93].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
sub_38();
} else {
sub_39();
}
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_38() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
return false;
}
bool sub_39() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_40();
} else {
sub_41();
}
return false;
}
bool sub_40() {
reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
return false;
}
bool sub_41() {
reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
return false;
}
bool sub_46() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_48() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_49() {
reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
if (uniforms.b[10]) {
sub_50();
} else {
sub_51();
}
return false;
}
bool sub_50() {
sub_37();
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_51() {
if (uniforms.b[13]) {
sub_52();
} else {
sub_55();
}
return false;
}
bool sub_52() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_53();
} else {
sub_54();
}
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_53() {
reg_tmp6 = reg_tmp10;
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
reg_tmp6.w = rcp_s(reg_tmp4.z);
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
return false;
}
bool sub_54() {
sub_48();
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
return false;
}
bool sub_55() {
vs_out_attr5 = uniforms.f[93].xxxx;
return false;
}
bool sub_56() {
reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
if (uniforms.b[11]) {
sub_57();
} else {
sub_58();
}
return false;
}
bool sub_57() {
sub_37();
reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
vs_out_attr6 = reg_tmp5;
return false;
}
bool sub_58() {
if (uniforms.b[14]) {
sub_59();
} else {
sub_60();
}
return false;
}
bool sub_59() {
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp5.zw = (reg_tmp6.zwww).zw;
sub_48();
reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
vs_out_attr6 = reg_tmp5;
return false;
}
bool sub_60() {
vs_out_attr6 = uniforms.f[93].xxxx;
return false;
}
// reference: D861684484B7142A, BDB4CD9AD6AFB6D9
// shader: 8B30, CDEB5869290EB93A
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rrr) * (texcolor1.rrr), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.ggg) * (rounded_primary_color.ggg) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor1.bbb) * (rounded_primary_color.bbb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: 08FDBC5D45DA7BBA, CDEB5869290EB93A
// program: BDB4CD9AD6AFB6D9, EADA2116091C5E01, CDEB5869290EB93A
// shader: 8B30, D997ADDEB28E7E9E
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
last_tex_env_out = vec4(color_output_1, alpha_output_1);
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
// reference: 5D7ABB824C4BF6D5, D997ADDEB28E7E9E
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, D997ADDEB28E7E9E
// shader: 8B30, 2B1D166E58DA6717
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
// reference: 8CA598A516BF13EE, 2B1D166E58DA6717
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 2B1D166E58DA6717
// reference: 5D7ABB820B58002E, D997ADDEB28E7E9E
// reference: 0DC9D15E0B58002E, D997ADDEB28E7E9E
// shader: 8B30, C4DC9EA3C88D43A2
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
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTSigned(6, dot(normal, normalize(half_vector))));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTSigned(6, dot(normal, normalize(half_vector))));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((primary_fragment_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (texcolor0.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a < alphatest_ref) discard;
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
// reference: 33F0D604A108B828, C4DC9EA3C88D43A2
// program: 0000000000000000, 0000000000000000, C4DC9EA3C88D43A2
// shader: 8DD9, F777123A8C3D5E20

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
// reference: 46A0C2E6B155D5CD, F777123A8C3D5E20
// shader: 8B31, 4E7D985E0EC030BF

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
layout(location=8) in vec4 vs_in_reg8;

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

bool sub_19();
bool sub_4();
bool sub_8();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_18();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_5();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_41();
bool sub_42();
bool sub_43();
bool sub_44();
bool sub_46();
bool sub_36();
bool sub_37();
bool sub_38();
bool sub_39();
bool sub_40();
bool sub_45();
bool sub_47();
bool sub_48();
bool sub_49();
bool sub_50();
bool sub_51();
bool sub_52();
bool sub_53();
bool sub_54();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_19() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_8() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_21();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_3();
} else {
sub_6();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_20();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_3() {
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_4();
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
sub_5();
return false;
}
bool sub_6() {
if (all(bool_regs)) {
sub_7();
} else {
sub_18();
}
return false;
}
bool sub_7() {
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_8();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_8();
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
sub_9();
return false;
}
bool sub_18() {
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_19();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_19();
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
bool sub_20() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_21() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_22();
} else {
sub_23();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_24();
} else {
sub_25();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_28();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_22() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_23() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_24() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_5();
return false;
}
bool sub_25() {
if (all(bool_regs)) {
sub_26();
} else {
sub_27();
}
return false;
}
bool sub_26() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_9();
return false;
}
bool sub_27() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_28() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_5() {
uint jmp_to = 191u;
while (true) {
switch (jmp_to) {
case 191u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 207u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 207u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 207u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_9() {
uint jmp_to = 208u;
while (true) {
switch (jmp_to) {
case 208u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 283u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 245u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 283u; break;
}
case 245u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_10();
} else {
sub_15();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 283u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_10() {
if (bool_regs.y) {
sub_11();
} else {
sub_12();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_11() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_12() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_13();
} else {
sub_14();
}
return false;
}
bool sub_13() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_14() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_15() {
if (bool_regs.y) {
sub_16();
} else {
sub_17();
}
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_17() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_29() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
reg_tmp9.w = (uniforms.f[21].wwww).w;
if (bool_regs.y) {
sub_30();
}
sub_32();
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_30() {
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (uniforms.b[7]) {
sub_31();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_31() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_32() {
reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[24].wwww;
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp3 = uniforms.f[22];
reg_tmp2 = uniforms.f[23] + -reg_tmp3;
reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
if (uniforms.b[6]) {
sub_33();
}
reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_33() {
reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
return false;
}
bool sub_34() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_35();
} else {
sub_41();
}
return false;
}
bool sub_35() {
sub_36();
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_41() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_42();
} else {
sub_43();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_42() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_43() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_44();
} else {
sub_46();
}
return false;
}
bool sub_44() {
sub_45();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_46() {
sub_47();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_36() {
bool_regs = equal(uniforms.f[93].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
sub_37();
} else {
sub_38();
}
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_37() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
return false;
}
bool sub_38() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_39();
} else {
sub_40();
}
return false;
}
bool sub_39() {
reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
return false;
}
bool sub_40() {
reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
return false;
}
bool sub_45() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_47() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_48() {
reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
if (uniforms.b[10]) {
sub_49();
} else {
sub_50();
}
return false;
}
bool sub_49() {
sub_36();
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_50() {
if (uniforms.b[13]) {
sub_51();
} else {
sub_54();
}
return false;
}
bool sub_51() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_52();
} else {
sub_53();
}
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_52() {
reg_tmp6 = reg_tmp10;
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
reg_tmp6.w = rcp_s(reg_tmp4.z);
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
return false;
}
bool sub_53() {
sub_47();
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
return false;
}
bool sub_54() {
vs_out_attr5 = uniforms.f[93].xxxx;
return false;
}
bool sub_0() {
sub_1();
sub_29();
sub_34();
sub_48();
return true;
}
// reference: 0C63AAAC5AB072C8, 4E7D985E0EC030BF
// shader: 8B30, B45500331959E4A8
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
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: 87064A30C35BFC52, B45500331959E4A8
// program: 4E7D985E0EC030BF, F777123A8C3D5E20, B45500331959E4A8
// shader: 8B31, A12CD4E91FC52B7E

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
layout(location=8) in vec4 vs_in_reg8;

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

bool sub_21();
bool sub_4();
bool sub_9();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_5();
bool sub_7();
bool sub_8();
bool sub_10();
bool sub_20();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_6();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_36();
bool sub_37();
bool sub_38();
bool sub_44();
bool sub_45();
bool sub_46();
bool sub_47();
bool sub_49();
bool sub_39();
bool sub_40();
bool sub_41();
bool sub_42();
bool sub_43();
bool sub_48();
bool sub_50();
bool sub_51();
bool sub_52();
bool sub_53();
bool sub_54();
bool sub_55();
bool sub_56();
bool sub_57();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_21() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_9() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_24();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_3();
} else {
sub_7();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_23();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_3() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_4();
}
if (uniforms.b[8]) {
sub_5();
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
sub_6();
return false;
}
bool sub_5() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_4();
}
return false;
}
bool sub_7() {
if (all(bool_regs)) {
sub_8();
} else {
sub_20();
}
return false;
}
bool sub_8() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_9();
}
if (uniforms.b[8]) {
sub_10();
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
sub_11();
return false;
}
bool sub_10() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_9();
}
return false;
}
bool sub_20() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_21();
}
if (uniforms.b[8]) {
sub_22();
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
bool sub_22() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_21();
}
return false;
}
bool sub_23() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_24() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_25();
} else {
sub_26();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_27();
} else {
sub_28();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_31();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_25() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_26() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_27() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_6();
return false;
}
bool sub_28() {
if (all(bool_regs)) {
sub_29();
} else {
sub_30();
}
return false;
}
bool sub_29() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_11();
return false;
}
bool sub_30() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_31() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_6() {
uint jmp_to = 218u;
while (true) {
switch (jmp_to) {
case 218u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 234u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 234u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 234u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_11() {
uint jmp_to = 235u;
while (true) {
switch (jmp_to) {
case 235u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 310u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 272u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 310u; break;
}
case 272u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_12();
} else {
sub_17();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 310u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_12() {
if (bool_regs.y) {
sub_13();
} else {
sub_14();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_13() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_14() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_15();
} else {
sub_16();
}
return false;
}
bool sub_15() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_17() {
if (bool_regs.y) {
sub_18();
} else {
sub_19();
}
return false;
}
bool sub_18() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_19() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_32() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
reg_tmp9.w = (uniforms.f[21].wwww).w;
if (bool_regs.y) {
sub_33();
}
sub_35();
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_33() {
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (uniforms.b[7]) {
sub_34();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_34() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_35() {
reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[24].wwww;
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp3 = uniforms.f[22];
reg_tmp2 = uniforms.f[23] + -reg_tmp3;
reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
if (uniforms.b[6]) {
sub_36();
}
reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_36() {
reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
return false;
}
bool sub_37() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_38();
} else {
sub_44();
}
return false;
}
bool sub_38() {
sub_39();
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_44() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_45();
} else {
sub_46();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_45() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_46() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_47();
} else {
sub_49();
}
return false;
}
bool sub_47() {
sub_48();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_49() {
sub_50();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_39() {
bool_regs = equal(uniforms.f[93].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
sub_40();
} else {
sub_41();
}
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_40() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
return false;
}
bool sub_41() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_42();
} else {
sub_43();
}
return false;
}
bool sub_42() {
reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
return false;
}
bool sub_43() {
reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
return false;
}
bool sub_48() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_50() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_51() {
reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
if (uniforms.b[10]) {
sub_52();
} else {
sub_53();
}
return false;
}
bool sub_52() {
sub_39();
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_53() {
if (uniforms.b[13]) {
sub_54();
} else {
sub_57();
}
return false;
}
bool sub_54() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_55();
} else {
sub_56();
}
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_55() {
reg_tmp6 = reg_tmp10;
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
reg_tmp6.w = rcp_s(reg_tmp4.z);
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
return false;
}
bool sub_56() {
sub_50();
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
return false;
}
bool sub_57() {
vs_out_attr5 = uniforms.f[93].xxxx;
return false;
}
bool sub_0() {
sub_1();
sub_32();
sub_37();
sub_51();
return true;
}
// reference: C734409B6DC443A9, A12CD4E91FC52B7E
// shader: 8B30, A90933BCAD7EDACA
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
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: C5BB69E0D8E9BE55, A90933BCAD7EDACA
// program: A12CD4E91FC52B7E, F777123A8C3D5E20, A90933BCAD7EDACA
// program: A12CD4E91FC52B7E, F777123A8C3D5E20, B45500331959E4A8
// shader: 8B30, F8F2A39CE76E8AB7
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
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: E160AD7A786E0640, F8F2A39CE76E8AB7
// program: 4E7D985E0EC030BF, F777123A8C3D5E20, F8F2A39CE76E8AB7
// shader: 8B31, 1CFEE28FC79B4D90

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
bool sub_19();
bool sub_4();
bool sub_8();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_18();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_5();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_41();
bool sub_42();
bool sub_43();
bool sub_44();
bool sub_46();
bool sub_36();
bool sub_37();
bool sub_38();
bool sub_39();
bool sub_40();
bool sub_45();
bool sub_47();
bool sub_48();
bool sub_49();
bool sub_50();
bool sub_51();
bool sub_52();
bool sub_53();
bool sub_54();
bool sub_55();
bool sub_56();
bool sub_57();
bool sub_58();
bool sub_59();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
sub_1();
sub_29();
sub_34();
sub_48();
sub_55();
return true;
}
bool sub_19() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_8() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_21();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_3();
} else {
sub_6();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_20();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_3() {
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_4();
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
sub_5();
return false;
}
bool sub_6() {
if (all(bool_regs)) {
sub_7();
} else {
sub_18();
}
return false;
}
bool sub_7() {
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_8();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_8();
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
sub_9();
return false;
}
bool sub_18() {
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_19();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_19();
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
bool sub_20() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_21() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_22();
} else {
sub_23();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_24();
} else {
sub_25();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_28();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_22() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_23() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_24() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_5();
return false;
}
bool sub_25() {
if (all(bool_regs)) {
sub_26();
} else {
sub_27();
}
return false;
}
bool sub_26() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_9();
return false;
}
bool sub_27() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_28() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_5() {
uint jmp_to = 191u;
while (true) {
switch (jmp_to) {
case 191u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 207u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 207u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 207u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_9() {
uint jmp_to = 208u;
while (true) {
switch (jmp_to) {
case 208u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 283u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 245u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 283u; break;
}
case 245u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_10();
} else {
sub_15();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 283u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_10() {
if (bool_regs.y) {
sub_11();
} else {
sub_12();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_11() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_12() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_13();
} else {
sub_14();
}
return false;
}
bool sub_13() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_14() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_15() {
if (bool_regs.y) {
sub_16();
} else {
sub_17();
}
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_17() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_29() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
reg_tmp9.w = (uniforms.f[21].wwww).w;
if (bool_regs.y) {
sub_30();
}
sub_32();
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_30() {
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (uniforms.b[7]) {
sub_31();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_31() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_32() {
reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[24].wwww;
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp3 = uniforms.f[22];
reg_tmp2 = uniforms.f[23] + -reg_tmp3;
reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
if (uniforms.b[6]) {
sub_33();
}
reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_33() {
reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
return false;
}
bool sub_34() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_35();
} else {
sub_41();
}
return false;
}
bool sub_35() {
sub_36();
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_41() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_42();
} else {
sub_43();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_42() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_43() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_44();
} else {
sub_46();
}
return false;
}
bool sub_44() {
sub_45();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_46() {
sub_47();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_36() {
bool_regs = equal(uniforms.f[93].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
sub_37();
} else {
sub_38();
}
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_37() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
return false;
}
bool sub_38() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_39();
} else {
sub_40();
}
return false;
}
bool sub_39() {
reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
return false;
}
bool sub_40() {
reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
return false;
}
bool sub_45() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_47() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_48() {
reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
if (uniforms.b[10]) {
sub_49();
} else {
sub_50();
}
return false;
}
bool sub_49() {
sub_36();
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_50() {
if (uniforms.b[13]) {
sub_51();
} else {
sub_54();
}
return false;
}
bool sub_51() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_52();
} else {
sub_53();
}
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_52() {
reg_tmp6 = reg_tmp10;
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
reg_tmp6.w = rcp_s(reg_tmp4.z);
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
return false;
}
bool sub_53() {
sub_47();
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
return false;
}
bool sub_54() {
vs_out_attr5 = uniforms.f[93].xxxx;
return false;
}
bool sub_55() {
reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
if (uniforms.b[11]) {
sub_56();
} else {
sub_57();
}
return false;
}
bool sub_56() {
sub_36();
reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
vs_out_attr6 = reg_tmp5;
return false;
}
bool sub_57() {
if (uniforms.b[14]) {
sub_58();
} else {
sub_59();
}
return false;
}
bool sub_58() {
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp5.zw = (reg_tmp6.zwww).zw;
sub_47();
reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
vs_out_attr6 = reg_tmp5;
return false;
}
bool sub_59() {
vs_out_attr6 = uniforms.f[93].xxxx;
return false;
}
// reference: 1B9B5F8EC053C89A, 1CFEE28FC79B4D90
// shader: 8B30, 83D4A737307364BB
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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.aaa) + (texcolor2.rgb) * (vec3(1) - (texcolor1.aaa)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: 1520D334C7F08FC0, 83D4A737307364BB
// program: 1CFEE28FC79B4D90, EADA2116091C5E01, 83D4A737307364BB
// program: A12CD4E91FC52B7E, F777123A8C3D5E20, F8F2A39CE76E8AB7
// shader: 8B31, CAFBCE2510D30131

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
bool sub_21();
bool sub_4();
bool sub_9();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_5();
bool sub_7();
bool sub_8();
bool sub_10();
bool sub_20();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_6();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_36();
bool sub_37();
bool sub_38();
bool sub_44();
bool sub_45();
bool sub_46();
bool sub_47();
bool sub_49();
bool sub_39();
bool sub_40();
bool sub_41();
bool sub_42();
bool sub_43();
bool sub_48();
bool sub_50();
bool sub_51();
bool sub_52();
bool sub_53();
bool sub_54();
bool sub_55();
bool sub_56();
bool sub_57();
bool sub_58();
bool sub_59();
bool sub_60();
bool sub_61();
bool sub_62();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
sub_1();
sub_32();
sub_37();
sub_51();
sub_58();
return true;
}
bool sub_21() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_9() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_24();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_3();
} else {
sub_7();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_23();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_3() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_4();
}
if (uniforms.b[8]) {
sub_5();
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
sub_6();
return false;
}
bool sub_5() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_4();
}
return false;
}
bool sub_7() {
if (all(bool_regs)) {
sub_8();
} else {
sub_20();
}
return false;
}
bool sub_8() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_9();
}
if (uniforms.b[8]) {
sub_10();
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
sub_11();
return false;
}
bool sub_10() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_9();
}
return false;
}
bool sub_20() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_21();
}
if (uniforms.b[8]) {
sub_22();
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
bool sub_22() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_21();
}
return false;
}
bool sub_23() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_24() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_25();
} else {
sub_26();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_27();
} else {
sub_28();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_31();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_25() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_26() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_27() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_6();
return false;
}
bool sub_28() {
if (all(bool_regs)) {
sub_29();
} else {
sub_30();
}
return false;
}
bool sub_29() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_11();
return false;
}
bool sub_30() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_31() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_6() {
uint jmp_to = 218u;
while (true) {
switch (jmp_to) {
case 218u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 234u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 234u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 234u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_11() {
uint jmp_to = 235u;
while (true) {
switch (jmp_to) {
case 235u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 310u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 272u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 310u; break;
}
case 272u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_12();
} else {
sub_17();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 310u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_12() {
if (bool_regs.y) {
sub_13();
} else {
sub_14();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_13() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_14() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_15();
} else {
sub_16();
}
return false;
}
bool sub_15() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_17() {
if (bool_regs.y) {
sub_18();
} else {
sub_19();
}
return false;
}
bool sub_18() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_19() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_32() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
reg_tmp9.w = (uniforms.f[21].wwww).w;
if (bool_regs.y) {
sub_33();
}
sub_35();
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_33() {
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (uniforms.b[7]) {
sub_34();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_34() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_35() {
reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[24].wwww;
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp3 = uniforms.f[22];
reg_tmp2 = uniforms.f[23] + -reg_tmp3;
reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
if (uniforms.b[6]) {
sub_36();
}
reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_36() {
reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
return false;
}
bool sub_37() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_38();
} else {
sub_44();
}
return false;
}
bool sub_38() {
sub_39();
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_44() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_45();
} else {
sub_46();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_45() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_46() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_47();
} else {
sub_49();
}
return false;
}
bool sub_47() {
sub_48();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_49() {
sub_50();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_39() {
bool_regs = equal(uniforms.f[93].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
sub_40();
} else {
sub_41();
}
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_40() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
return false;
}
bool sub_41() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_42();
} else {
sub_43();
}
return false;
}
bool sub_42() {
reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
return false;
}
bool sub_43() {
reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
return false;
}
bool sub_48() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_50() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_51() {
reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
if (uniforms.b[10]) {
sub_52();
} else {
sub_53();
}
return false;
}
bool sub_52() {
sub_39();
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_53() {
if (uniforms.b[13]) {
sub_54();
} else {
sub_57();
}
return false;
}
bool sub_54() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_55();
} else {
sub_56();
}
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_55() {
reg_tmp6 = reg_tmp10;
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
reg_tmp6.w = rcp_s(reg_tmp4.z);
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
return false;
}
bool sub_56() {
sub_50();
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
return false;
}
bool sub_57() {
vs_out_attr5 = uniforms.f[93].xxxx;
return false;
}
bool sub_58() {
reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
if (uniforms.b[11]) {
sub_59();
} else {
sub_60();
}
return false;
}
bool sub_59() {
sub_39();
reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
vs_out_attr6 = reg_tmp5;
return false;
}
bool sub_60() {
if (uniforms.b[14]) {
sub_61();
} else {
sub_62();
}
return false;
}
bool sub_61() {
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp5.zw = (reg_tmp6.zwww).zw;
sub_50();
reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
vs_out_attr6 = reg_tmp5;
return false;
}
bool sub_62() {
vs_out_attr6 = uniforms.f[93].xxxx;
return false;
}
// reference: D0CCB5B99557C931, CAFBCE2510D30131
// program: CAFBCE2510D30131, EADA2116091C5E01, 83D4A737307364BB
// shader: 8B30, 1505A76486D560D8
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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (const_color[0].aaa) + (texcolor2.rgb) * (vec3(1) - (const_color[0].aaa)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].r) * (const_color[0].a) + (texcolor2.a) * (1.0 - (const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor1.aaa) * (secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: E8F94841C617919C, 1505A76486D560D8
// program: CAFBCE2510D30131, EADA2116091C5E01, 1505A76486D560D8
// shader: 8B30, 301CE98127309CFB
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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (const_color[0].aaa) + (texcolor2.rgb) * (vec3(1) - (const_color[0].aaa)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: D6625204F2B04CFB, 301CE98127309CFB
// program: CAFBCE2510D30131, EADA2116091C5E01, 301CE98127309CFB
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
// shader: 8B31, 550FDE26A285702A

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
bool sub_15();
bool sub_2();
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
bool sub_0();
bool sub_3();
bool sub_4();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xyz = (vs_in_reg0.xyzz).xyz;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp11.xy = (vs_in_reg1.xyyy).xy;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_15() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = mul_s(vs_in_reg2, reg_tmp13);
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
reg_tmp2.xy = (mul_s(uniforms.f[2].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, reg_tmp2.xyyy)).xy;
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
reg_tmp5.x = (uniforms.f[5].wwww).x;
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_7();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_8();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_9();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_10();
}
reg_tmp3 = reg_tmp14.xyxy;
reg_tmp3.y = (uniforms.f[0].zzzz + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[0].zzzz + -reg_tmp3.wwww).w;
reg_tmp2 = mul_s(uniforms.f[83].xxyy, reg_tmp3);
reg_tmp2 = -uniforms.f[0].yyyy + reg_tmp2;
reg_tmp2 = mul_s(reg_tmp2, reg_tmp1.xyyx);
reg_tmp2.x = (reg_tmp2.xxxx + -reg_tmp2.yyyy).x;
reg_tmp2.y = (reg_tmp2.zzzz + reg_tmp2.wwww).y;
reg_tmp4.xy = (uniforms.f[3].xyyy).xy;
reg_tmp3.xy = (fma_s(reg_tmp2.xyyy, uniforms.f[3].zwww, reg_tmp4.xyyy)).xy;
reg_tmp3.xy = (uniforms.f[0].yyyy + reg_tmp3.xyyy).xy;
reg_tmp11.xy = (reg_tmp3.xyyy).xy;
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
reg_tmp5.x = (uniforms.f[6].wwww).x;
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
reg_tmp3 = reg_tmp14.xyxy;
reg_tmp3.y = (uniforms.f[0].zzzz + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[0].zzzz + -reg_tmp3.wwww).w;
reg_tmp2 = mul_s(uniforms.f[83].zzww, reg_tmp3);
reg_tmp2 = -uniforms.f[0].yyyy + reg_tmp2;
reg_tmp2 = mul_s(reg_tmp2, reg_tmp1.xyyx);
reg_tmp2.x = (reg_tmp2.xxxx + -reg_tmp2.yyyy).x;
reg_tmp2.y = (reg_tmp2.zzzz + reg_tmp2.wwww).y;
reg_tmp4.xy = (uniforms.f[4].xyyy).xy;
reg_tmp3.xy = (fma_s(reg_tmp2.xyyy, uniforms.f[4].zwww, reg_tmp4.xyyy)).xy;
reg_tmp3.xy = (uniforms.f[0].yyyy + reg_tmp3.xyyy).xy;
reg_tmp12.xy = (reg_tmp3.xyyy).xy;
return false;
}
bool sub_7() {
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
return false;
}
bool sub_8() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
return false;
}
bool sub_9() {
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_10() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_11() {
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
return false;
}
bool sub_12() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
return false;
}
bool sub_13() {
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_14() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xyz = (mul_s(uniforms.f[2].xyxx, reg_tmp10.xyzz)).xyz;
reg_tmp14 = reg_tmp10;
reg_tmp0 = uniforms.f[7];
sub_2();
reg_tmp10.x = dot_s(reg_tmp14, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp14, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp14, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp14, reg_tmp9);
reg_tmp6 = uniforms.f[8];
reg_tmp7 = uniforms.f[9];
reg_tmp8 = uniforms.f[10];
reg_tmp9 = uniforms.f[0].xxxz;
reg_tmp6.w = (uniforms.f[5].xxxx).w;
reg_tmp7.w = (uniforms.f[5].yyyy).w;
reg_tmp8.w = (uniforms.f[5].zzzz).w;
reg_tmp2.x = dot_s(reg_tmp10, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp10, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp10, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp10, reg_tmp9);
reg_tmp10 = reg_tmp2;
reg_tmp3.xyz = (uniforms.f[5].xyzz).xyz;
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
reg_tmp2 = uniforms.f[1];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_6();
sub_15();
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
// reference: 0995A72BC7729DC9, 550FDE26A285702A
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
// program: 550FDE26A285702A, D9AC2D854F4D2F3B, B8762D381189F738
// shader: 8B30, 5FF2DB80DA8B41EA
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
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
// reference: A1E30557A974AB22, 5FF2DB80DA8B41EA
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 5FF2DB80DA8B41EA
// shader: 8B31, B10263C48E967D5C

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

bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
reg_tmp0 = vs_in_reg0;
reg_tmp0.w = (uniforms.f[78].yyyy).w;
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp0);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp0);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp0);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp0);
vs_out_attr1 = mul_s(uniforms.f[77], vs_in_reg1);
vs_out_attr2 = vs_in_reg2;
vs_out_attr3 = vs_in_reg3;
vs_out_attr4 = vs_in_reg4;
return true;
}
// reference: 3D2105050294DB20, B10263C48E967D5C
// shader: 8B30, 6169D6D4DAFA7530
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
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (const_color[3].rgb) + (texcolor0.rgb) * (vec3(1) - (const_color[3].rgb)), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((1.0 - texcolor1.a) * (const_color[3].a) + (1.0 - texcolor0.a) * (1.0 - (const_color[3].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor2.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[4].rgb)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor2.a) * (const_color[4].a) + (last_tex_env_out.a) * (1.0 - (const_color[4].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C1B67FFC91F1C944, 6169D6D4DAFA7530
// program: B10263C48E967D5C, B6B95AFD9466EC70, 6169D6D4DAFA7530
// shader: 8B30, DA7B2B872C41591A
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
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (const_color[3].rgb) + (texcolor0.rgb) * (vec3(1) - (const_color[3].rgb)), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((1.0 - texcolor1.a) * (const_color[3].a) + (1.0 - texcolor0.a) * (1.0 - (const_color[3].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor2.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[4].rgb)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor2.a) * (const_color[4].a) + (last_tex_env_out.a) * (1.0 - (const_color[4].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (texcolor0.rgb);
float alpha_output_5 = (texcolor0.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 04E5A0907C58CE89, DA7B2B872C41591A
// program: B10263C48E967D5C, B6B95AFD9466EC70, DA7B2B872C41591A
// shader: 8B30, 1E93E3E2D2D9704F
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
vec3 color_output_0 = byteround(clamp((texcolor1.aaa) * (const_color[0].rgb) + (texcolor1.aaa), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (last_tex_env_out.rgb) + (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (const_color[1].a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rrr) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (const_color[2].a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((combiner_buffer.ggg) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (const_color[3].a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.bbb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1DD627DB37C43360, 1E93E3E2D2D9704F
// program: B10263C48E967D5C, B6B95AFD9466EC70, 1E93E3E2D2D9704F
// shader: 8DD9, 58F5ACF9B36D7DF3

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
// reference: 3901EC4BEC56958E, 58F5ACF9B36D7DF3
// shader: 8B31, 38CAD145F73DC842

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
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

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

bool sub_20();
bool sub_3();
bool sub_8();
bool sub_1();
bool sub_2();
bool sub_4();
bool sub_6();
bool sub_7();
bool sub_9();
bool sub_19();
bool sub_21();
bool sub_22();
bool sub_5();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_20() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_3() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_8() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_2();
} else {
sub_6();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_22();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_2() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_3();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_3();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_3();
}
if (uniforms.b[8]) {
sub_4();
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
sub_5();
return false;
}
bool sub_4() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_3();
}
return false;
}
bool sub_6() {
if (all(bool_regs)) {
sub_7();
} else {
sub_19();
}
return false;
}
bool sub_7() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_8();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_8();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_8();
}
if (uniforms.b[8]) {
sub_9();
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
sub_10();
return false;
}
bool sub_9() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_8();
}
return false;
}
bool sub_19() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_20();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_20();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_20();
}
if (uniforms.b[8]) {
sub_21();
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
bool sub_21() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_20();
}
return false;
}
bool sub_22() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_5() {
uint jmp_to = 145u;
while (true) {
switch (jmp_to) {
case 145u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 161u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 161u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 161u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_10() {
uint jmp_to = 162u;
while (true) {
switch (jmp_to) {
case 162u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 237u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 199u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 237u; break;
}
case 199u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_11();
} else {
sub_16();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 237u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_11() {
if (bool_regs.y) {
sub_12();
} else {
sub_13();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_12() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_13() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_14();
} else {
sub_15();
}
return false;
}
bool sub_14() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_15() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_16() {
if (bool_regs.y) {
sub_17();
} else {
sub_18();
}
return false;
}
bool sub_17() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_18() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_23() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9 = uniforms.f[21];
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (bool_regs.y) {
sub_24();
}
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_24() {
if (uniforms.b[7]) {
sub_25();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_25() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_0() {
sub_1();
sub_23();
return true;
}
// reference: 8DD392D3870F668C, 38CAD145F73DC842
// shader: 8B30, 02EDC449C1912C30
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
vec3 color_output_0 = (primary_fragment_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 00310758F02BE887, 02EDC449C1912C30
// program: 38CAD145F73DC842, 58F5ACF9B36D7DF3, 02EDC449C1912C30
// shader: 8B30, 5FC3D10768C6B23E
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
vec3 color_output_0 = (primary_fragment_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = byteround(clamp((texcolor1.rrr) * (const_color[4].rrr) + (texcolor0.rrr) * (vec3(1) - (const_color[4].rrr)), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((texcolor2.rrr) * (const_color[5].rrr) + (last_tex_env_out.ggg) * (vec3(1) - (const_color[5].rrr)), vec3(0), vec3(1)));
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1C42E1B33DF57A7D, 5FC3D10768C6B23E
// program: B10263C48E967D5C, B6B95AFD9466EC70, 5FC3D10768C6B23E
// shader: 8B30, CD93612DC954AC07
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
vec3 color_output_0 = (primary_fragment_color.rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = byteround(clamp((texcolor1.rrr) * (const_color[4].rrr) + (texcolor0.rrr) * (vec3(1) - (const_color[4].rrr)), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb) + (texcolor2.ggg) * (vec3(1) - (const_color[5].rgb)), vec3(0), vec3(1)));
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 67D33CC207456605, CD93612DC954AC07
// program: B10263C48E967D5C, B6B95AFD9466EC70, CD93612DC954AC07
// reference: 2460550AF76FBB71, 9919954F6740CFCD
// shader: 8B30, 06A4A2169F9065FE
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
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E1B117184DDC1881, 06A4A2169F9065FE
// program: A150FC784311F5E1, B6B95AFD9466EC70, 06A4A2169F9065FE
// reference: DC16F27916BF13EE, 2B1D166E58DA6717
// reference: D6A06F470A065D40, 5FF2DB80DA8B41EA
// shader: 8B30, 78CD8FC0579E0A56
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
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
// reference: 077F4C6017E14E80, 78CD8FC0579E0A56
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 78CD8FC0579E0A56
// shader: 8B31, 3A1D4E07C99F24D3

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
layout(location=7) in vec4 vs_in_reg7;

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
bool sub_6();
bool sub_7();
bool sub_17();
bool sub_18();
bool sub_5();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_29();
bool sub_28();
bool sub_30();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_2();
} else {
sub_3();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_4();
} else {
sub_6();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_18();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_2() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_3() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_4() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_5();
return false;
}
bool sub_6() {
if (all(bool_regs)) {
sub_7();
} else {
sub_17();
}
return false;
}
bool sub_7() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_8();
return false;
}
bool sub_17() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_18() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_5() {
uint jmp_to = 83u;
while (true) {
switch (jmp_to) {
case 83u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 99u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 99u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 99u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_8() {
uint jmp_to = 100u;
while (true) {
switch (jmp_to) {
case 100u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 175u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 137u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 175u; break;
}
case 137u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_9();
} else {
sub_14();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 175u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_9() {
if (bool_regs.y) {
sub_10();
} else {
sub_11();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_11() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_12();
} else {
sub_13();
}
return false;
}
bool sub_12() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_13() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_14() {
if (bool_regs.y) {
sub_15();
} else {
sub_16();
}
return false;
}
bool sub_15() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_19() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9 = uniforms.f[21];
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (bool_regs.y) {
sub_20();
}
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_20() {
if (uniforms.b[7]) {
sub_21();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_21() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_22() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_23();
} else {
sub_24();
}
return false;
}
bool sub_23() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_24() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_25();
} else {
sub_26();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_25() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_26() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_27();
} else {
sub_29();
}
return false;
}
bool sub_27() {
sub_28();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_29() {
sub_30();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_28() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_30() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_0() {
sub_1();
sub_19();
sub_22();
return true;
}
// reference: 37324CFDC897B07C, 3A1D4E07C99F24D3
// shader: 8B30, B64388AC867F9458
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
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F1CDFEFBBE985357, B64388AC867F9458
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, B64388AC867F9458
// reference: BFD964220A065D40, 5FF2DB80DA8B41EA
// shader: 8B30, 90564331A777EEC4
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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6E8591CA0764E659, 90564331A777EEC4
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 90564331A777EEC4
// reference: 6E06470517E14E80, 78CD8FC0579E0A56
// reference: 8613059B0A065D40, 5FF2DB80DA8B41EA
// shader: 8B30, 6E9907667E8550C4
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
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
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
// reference: C65E1BDA5937EE93, 6E9907667E8550C4
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 6E9907667E8550C4
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 9919954F6740CFCD
// shader: 8B30, E6D909CE3F49E823
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
last_tex_env_out = vec4(color_output_1, alpha_output_1);
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
// reference: 2460550A0B58002E, E6D909CE3F49E823
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, E6D909CE3F49E823
// shader: 8B30, 30615998ED243EB2
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor2.rrr) * (rounded_primary_color.rrr), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor2.ggg) * (rounded_primary_color.ggg) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor2.bbb) * (rounded_primary_color.bbb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.bbb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: 9764F76DE3DC9495, 30615998ED243EB2
// program: BDB4CD9AD6AFB6D9, EADA2116091C5E01, 30615998ED243EB2
// shader: 8B31, FA414C6D820CDC3E

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
layout(location=8) in vec4 vs_in_reg8;

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

bool sub_21();
bool sub_4();
bool sub_9();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_5();
bool sub_7();
bool sub_8();
bool sub_10();
bool sub_20();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_6();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_36();
bool sub_42();
bool sub_43();
bool sub_44();
bool sub_45();
bool sub_47();
bool sub_37();
bool sub_38();
bool sub_39();
bool sub_40();
bool sub_41();
bool sub_46();
bool sub_48();
bool sub_49();
bool sub_50();
bool sub_51();
bool sub_52();
bool sub_53();
bool sub_54();
bool sub_55();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_21() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_9() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_24();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_3();
} else {
sub_7();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_23();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_3() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_4();
}
if (uniforms.b[8]) {
sub_5();
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
sub_6();
return false;
}
bool sub_5() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_4();
}
return false;
}
bool sub_7() {
if (all(bool_regs)) {
sub_8();
} else {
sub_20();
}
return false;
}
bool sub_8() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_9();
}
if (uniforms.b[8]) {
sub_10();
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
sub_11();
return false;
}
bool sub_10() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_9();
}
return false;
}
bool sub_20() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_21();
}
if (uniforms.b[8]) {
sub_22();
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
bool sub_22() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_21();
}
return false;
}
bool sub_23() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_24() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_25();
} else {
sub_26();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_27();
} else {
sub_28();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_31();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_25() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_26() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_27() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_6();
return false;
}
bool sub_28() {
if (all(bool_regs)) {
sub_29();
} else {
sub_30();
}
return false;
}
bool sub_29() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_11();
return false;
}
bool sub_30() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_31() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_6() {
uint jmp_to = 218u;
while (true) {
switch (jmp_to) {
case 218u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 234u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 234u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 234u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_11() {
uint jmp_to = 235u;
while (true) {
switch (jmp_to) {
case 235u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 310u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 272u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 310u; break;
}
case 272u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_12();
} else {
sub_17();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 310u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_12() {
if (bool_regs.y) {
sub_13();
} else {
sub_14();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_13() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_14() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_15();
} else {
sub_16();
}
return false;
}
bool sub_15() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_17() {
if (bool_regs.y) {
sub_18();
} else {
sub_19();
}
return false;
}
bool sub_18() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_19() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_32() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9 = uniforms.f[21];
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (bool_regs.y) {
sub_33();
}
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_33() {
if (uniforms.b[7]) {
sub_34();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_34() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_35() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_36();
} else {
sub_42();
}
return false;
}
bool sub_36() {
sub_37();
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_42() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_43();
} else {
sub_44();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_43() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_44() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_45();
} else {
sub_47();
}
return false;
}
bool sub_45() {
sub_46();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_47() {
sub_48();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_37() {
bool_regs = equal(uniforms.f[93].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
sub_38();
} else {
sub_39();
}
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_38() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
return false;
}
bool sub_39() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_40();
} else {
sub_41();
}
return false;
}
bool sub_40() {
reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
return false;
}
bool sub_41() {
reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
return false;
}
bool sub_46() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_48() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_49() {
reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
if (uniforms.b[10]) {
sub_50();
} else {
sub_51();
}
return false;
}
bool sub_50() {
sub_37();
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_51() {
if (uniforms.b[13]) {
sub_52();
} else {
sub_55();
}
return false;
}
bool sub_52() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_53();
} else {
sub_54();
}
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_53() {
reg_tmp6 = reg_tmp10;
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
reg_tmp6.w = rcp_s(reg_tmp4.z);
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
return false;
}
bool sub_54() {
sub_48();
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
return false;
}
bool sub_55() {
vs_out_attr5 = uniforms.f[93].xxxx;
return false;
}
bool sub_0() {
sub_1();
sub_32();
sub_35();
sub_49();
return true;
}
// reference: CF999D666AFDDF2E, FA414C6D820CDC3E
// shader: 8B30, 3D0D7FCBAD2068B9
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: 3E9D240C5E0C6D7B, 3D0D7FCBAD2068B9
// program: FA414C6D820CDC3E, F777123A8C3D5E20, 3D0D7FCBAD2068B9
// reference: 6AE95EA34C4BF6D5, E6D909CE3F49E823
// reference: 70142FA5191F9B80, 3D0D7FCBAD2068B9
// reference: 3E9D240C191F9B80, 3D0D7FCBAD2068B9
// reference: 13F3B02B0B58002E, D997ADDEB28E7E9E
// reference: 4340DAF70B58002E, D997ADDEB28E7E9E
// reference: 6AE95EA30B58002E, E6D909CE3F49E823
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
// reference: 5A2AB705863B523C, 363919041B99023A
// program: A150FC784311F5E1, B6B95AFD9466EC70, 363919041B99023A
// shader: 8B30, 738AFEC9C05E0355
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
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6DB95224863B523C, 738AFEC9C05E0355
// program: A150FC784311F5E1, B6B95AFD9466EC70, 738AFEC9C05E0355
// reference: 6E2E4ED05E0C6D7B, 3D0D7FCBAD2068B9
// reference: 20A74579191F9B80, 3D0D7FCBAD2068B9
// reference: 3A5A347F4C4BF6D5, E6D909CE3F49E823
// reference: 6AE95EA32507F324, E6D909CE3F49E823
// shader: 8B30, 7F75D2E2820F55F7
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
vec3 color_output_0 = byteround(clamp((texcolor2.rgb) * (texcolor2.aaa) + (texcolor0.rgb) * (vec3(1) - (texcolor2.aaa)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor2.a) + (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((texcolor1.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (combiner_buffer.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) + (combiner_buffer.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp(min((rounded_primary_color.rgb) + (rounded_primary_color.rgb), vec3(1)) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) + (rounded_primary_color.rgb) - vec3(0.5), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CAEF718F023C8CC4, 7F75D2E2820F55F7
// program: A150FC784311F5E1, B6B95AFD9466EC70, 7F75D2E2820F55F7
// shader: 8B30, 8417FF1558C2C602
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
vec3 color_output_0 = byteround(clamp(min((rounded_primary_color.rgb) + (rounded_primary_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (rounded_primary_color.rgb) - vec3(0.5), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 17532FB598EE7DCC, 8417FF1558C2C602
// program: A150FC784311F5E1, B6B95AFD9466EC70, 8417FF1558C2C602
// shader: 8B30, C4570F55A3B18AB9
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((combiner_buffer.rgb) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
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
// reference: 9D24D6E4314F60DD, C4570F55A3B18AB9
// program: FA414C6D820CDC3E, F777123A8C3D5E20, C4570F55A3B18AB9
// shader: 8B30, 7C928704DCB60C15
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rrr) * (rounded_primary_color.rrr), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.ggg) * (rounded_primary_color.ggg) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor1.bbb) * (rounded_primary_color.bbb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (const_color[5].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: 7DF1836343BD45CE, 7C928704DCB60C15
// program: BDB4CD9AD6AFB6D9, EADA2116091C5E01, 7C928704DCB60C15
// reference: 2460550A1FE36D67, E6D909CE3F49E823
// shader: 8B30, 688E364FD3F17B12
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
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
// reference: 6AE95EA316BF13EE, 688E364FD3F17B12
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 688E364FD3F17B12
// reference: 6AE95EA31FE36D67, E6D909CE3F49E823
// shader: 8B30, 9483D4BD1E3F4648
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
// reference: BB367D84F110AE2A, 9483D4BD1E3F4648
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 9483D4BD1E3F4648
// reference: 3A5A347F16BF13EE, 688E364FD3F17B12
// shader: 8B30, B9D0C770DF65157F
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
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
// reference: 2460550ADCB524B8, B9D0C770DF65157F
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, B9D0C770DF65157F
// reference: 74D33FD6DCB524B8, B9D0C770DF65157F
// reference: 2460550AB5F92149, B9D0C770DF65157F
// shader: 8B30, 21526FC962D5B9D2
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
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + (refl_value * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((secondary_fragment_color.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (primary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 32C9AB80F4253BD4, 21526FC962D5B9D2
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 21526FC962D5B9D2
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 21526FC962D5B9D2
// reference: 7C40A029F4253BD4, 21526FC962D5B9D2
// shader: 8B30, 515DE3E73176BFC8
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
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
// reference: E54102B02C1F3E0A, 515DE3E73176BFC8
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 515DE3E73176BFC8
// reference: 2460550AC80E49F1, B9D0C770DF65157F
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, B9D0C770DF65157F
// shader: 8B30, DFC4CDAC5D20AAB7
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.ggg) * (texcolor1.ggg), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((rounded_primary_color.rrr) * (texcolor1.rrr) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
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
// reference: 2CA01C689C49B3DD, DFC4CDAC5D20AAB7
// program: FA414C6D820CDC3E, F777123A8C3D5E20, DFC4CDAC5D20AAB7
// shader: 8B30, 8C5438E98AD0BBD8
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
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
// reference: 13F3B02BC80E49F1, 8C5438E98AD0BBD8
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 8C5438E98AD0BBD8
// shader: 8B30, 6E551916C011C8AF
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - rounded_primary_color.aaa) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
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
// reference: F9127C20706107A8, 6E551916C011C8AF
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 6E551916C011C8AF
// reference: 6AE95EA3C80E49F1, B9D0C770DF65157F
// shader: 8B31, 8B16FF5D816EE414

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
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

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

bool sub_21();
bool sub_4();
bool sub_9();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_5();
bool sub_7();
bool sub_8();
bool sub_10();
bool sub_20();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_6();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_21() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_9() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_24();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2 = mul_s(uniforms.f[93].wwww, vs_in_reg7);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_3();
} else {
sub_7();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_23();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_3() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_4();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_4();
}
if (uniforms.b[8]) {
sub_5();
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
sub_6();
return false;
}
bool sub_5() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_4();
}
return false;
}
bool sub_7() {
if (all(bool_regs)) {
sub_8();
} else {
sub_20();
}
return false;
}
bool sub_8() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_9();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_9();
}
if (uniforms.b[8]) {
sub_10();
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
sub_11();
return false;
}
bool sub_10() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_9();
}
return false;
}
bool sub_20() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (reg_tmp2.xxxx).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.yyyy).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_21();
reg_tmp1.xy = (reg_tmp2.zzzz).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_21();
}
if (uniforms.b[8]) {
sub_22();
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
bool sub_22() {
reg_tmp1.xy = (reg_tmp2.wwww).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_21();
}
return false;
}
bool sub_23() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_24() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_25();
} else {
sub_26();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_27();
} else {
sub_28();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_31();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_25() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_26() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_27() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_6();
return false;
}
bool sub_28() {
if (all(bool_regs)) {
sub_29();
} else {
sub_30();
}
return false;
}
bool sub_29() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_11();
return false;
}
bool sub_30() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_31() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_6() {
uint jmp_to = 218u;
while (true) {
switch (jmp_to) {
case 218u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 234u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 234u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 234u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_11() {
uint jmp_to = 235u;
while (true) {
switch (jmp_to) {
case 235u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 310u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 272u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 310u; break;
}
case 272u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_12();
} else {
sub_17();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 310u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_12() {
if (bool_regs.y) {
sub_13();
} else {
sub_14();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_13() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_14() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_15();
} else {
sub_16();
}
return false;
}
bool sub_15() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_17() {
if (bool_regs.y) {
sub_18();
} else {
sub_19();
}
return false;
}
bool sub_18() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_19() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_32() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9 = uniforms.f[21];
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (bool_regs.y) {
sub_33();
}
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_33() {
if (uniforms.b[7]) {
sub_34();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_34() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_0() {
sub_1();
sub_32();
return true;
}
// reference: 57E99A0D442888B0, 8B16FF5D816EE414
// shader: 8B30, B43F46D5C9EB2EC2
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D582EB16A796EC33, B43F46D5C9EB2EC2
// program: 8B16FF5D816EE414, 58F5ACF9B36D7DF3, B43F46D5C9EB2EC2
// reference: 6AE95EA38F1DBF0A, B9D0C770DF65157F
// shader: 8B30, 9BF956A14F4794C4
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
vec3 color_output_0 = byteround(clamp((const_color[0].aaa) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 528D93E856CD1705, 9BF956A14F4794C4
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 9BF956A14F4794C4
// shader: 8B30, FC0D8A01C54D4809
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
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6E8591CA66DE188B, FC0D8A01C54D4809
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, FC0D8A01C54D4809
// shader: 8B30, 19B10F2B5B54FE82
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
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6E8591CA3777E9D7, 19B10F2B5B54FE82
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 19B10F2B5B54FE82
// reference: 4674B7F402C98D41, CDEB5869290EB93A
// reference: 6AE95EA358F09B9C, E6D909CE3F49E823
// reference: 8B3262499FFA48AE, A90933BCAD7EDACA
// shader: 8B30, 64B174ED1C56F246
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
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[1].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
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
// reference: 91CF134F0E831BFF, 64B174ED1C56F246
// program: A12CD4E91FC52B7E, F777123A8C3D5E20, 64B174ED1C56F246
// program: 1CFEE28FC79B4D90, EADA2116091C5E01, 301CE98127309CFB
// shader: 8B30, 9C83F4555B061C91
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
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
// reference: E1338A661EBD3009, 9C83F4555B061C91
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 9C83F4555B061C91
// reference: 884A81031EBD3009, 9C83F4555B061C91
// reference: 884A81030A065D40, 9C83F4555B061C91
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, E6D909CE3F49E823
// reference: 74D33FD60B58002E, E6D909CE3F49E823
// shader: 8B30, DF1A5EDF047DDE8B
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
// reference: F5BF762D16BF13EE, DF1A5EDF047DDE8B
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, DF1A5EDF047DDE8B
// shader: 8B30, E87C987687CFF8FE
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rrr) * (texcolor1.rrr), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor1.ggg) * (rounded_primary_color.ggg) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor1.bbb) * (rounded_primary_color.bbb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: 7DF18363090F3058, E87C987687CFF8FE
// program: BDB4CD9AD6AFB6D9, EADA2116091C5E01, E87C987687CFF8FE
// shader: 8B30, 1B2AD0E81111F738
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 2.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((texcolor2.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: 1E54F95E5D6F9C20, 1B2AD0E81111F738
// program: BDB4CD9AD6AFB6D9, EADA2116091C5E01, 1B2AD0E81111F738
// program: FA414C6D820CDC3E, F777123A8C3D5E20, 9483D4BD1E3F4648
// reference: 74D33FD61FE36D67, E6D909CE3F49E823
// shader: 8B30, 7806AC06CBA0B9B7
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((texcolor0.rgb) + (const_color[0].rgb), vec3(1)) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((texcolor1.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
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
// reference: 049554D98E66C96F, 7806AC06CBA0B9B7
// program: BDB4CD9AD6AFB6D9, EADA2116091C5E01, 7806AC06CBA0B9B7
// reference: 8CA598A551ACE515, 2B1D166E58DA6717
// shader: 8B30, 719B72EC07D65A02
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((texcolor0.aaa) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
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
// reference: E98CBE3C17F6EECF, 719B72EC07D65A02
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 719B72EC07D65A02
// reference: 4A1C5F708E66C96F, 7806AC06CBA0B9B7
// reference: F5BF762DF110AE2A, 9483D4BD1E3F4648
// reference: A50C1CF1F110AE2A, 9483D4BD1E3F4648
// reference: 32C9AB809D693E25, 21526FC962D5B9D2
// reference: B93FD4E017F6EECF, 719B72EC07D65A02
// shader: 8B30, E00089EF35D0A7D9
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor2.rrr) * (rounded_primary_color.rrr) + (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor2.ggg) * (rounded_primary_color.ggg) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor2.bbb) * (rounded_primary_color.bbb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
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
// reference: A1218F82839012FD, E00089EF35D0A7D9
// program: BDB4CD9AD6AFB6D9, EADA2116091C5E01, E00089EF35D0A7D9
// reference: EFA8842BC483E406, E00089EF35D0A7D9
// reference: C22C930C16BF13EE, 2B1D166E58DA6717
// reference: BB367D8451ACE515, DF1A5EDF047DDE8B
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 90564331A777EEC4
// shader: 8B30, E387886A357D63B8
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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E219756954196, E387886A357D63B8
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, E387886A357D63B8
// shader: 8B30, 462848B00895061F
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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 591674EB0764E659, 462848B00895061F
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 462848B00895061F
// shader: 8B30, 0299CA1AD527015F
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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9C7BE07D0764E659, 0299CA1AD527015F
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 0299CA1AD527015F
// shader: 8B30, 2B1175E8B76320C5
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
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) + (const_color[2].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (texcolor0.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BCF319C992DD2900, 2B1175E8B76320C5
// program: A150FC784311F5E1, B6B95AFD9466EC70, 2B1175E8B76320C5
// shader: 8B31, 298916613A5C4EB4

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
layout(location=7) in vec4 vs_in_reg7;

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
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_6();
bool sub_7();
bool sub_17();
bool sub_18();
bool sub_5();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_19();
bool sub_20();
bool sub_21();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_2();
} else {
sub_3();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_4();
} else {
sub_6();
}
vs_out_attr2 = -reg_tmp15;
reg_tmp0.x = dot_s(uniforms.f[86], reg_tmp15);
reg_tmp0.y = dot_s(uniforms.f[87], reg_tmp15);
reg_tmp0.z = dot_s(uniforms.f[88], reg_tmp15);
reg_tmp0.w = dot_s(uniforms.f[89], reg_tmp15);
reg_tmp1.x = (-reg_tmp0.wwww).x;
reg_tmp1.y = (mul_s(uniforms.f[95].zzzz, -reg_tmp0.wwww)).y;
bool_regs.x = reg_tmp0.xxxx.x > reg_tmp1.xyyy.x;
bool_regs.y = reg_tmp0.xxxx.y < reg_tmp1.xyyy.y;
if (all(bool_regs)) {
sub_18();
}
vs_out_attr0 = reg_tmp0;
return false;
}
bool sub_2() {
reg_tmp1.x = (mul_s(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_3() {
addr_regs.x = (ivec2(uniforms.f[93].xx)).x;
reg_tmp7.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_4() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_5();
return false;
}
bool sub_6() {
if (all(bool_regs)) {
sub_7();
} else {
sub_17();
}
return false;
}
bool sub_7() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.x].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.x].xyz, reg_tmp13.xyz);
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
sub_8();
return false;
}
bool sub_17() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_18() {
reg_tmp0.x = (-reg_tmp0.wwww).x;
return false;
}
bool sub_5() {
uint jmp_to = 83u;
while (true) {
switch (jmp_to) {
case 83u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 99u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 99u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 99u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_8() {
uint jmp_to = 100u;
while (true) {
switch (jmp_to) {
case 100u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[3]) {
jmp_to = 175u; break;
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
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 137u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 175u; break;
}
case 137u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_9();
} else {
sub_14();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 175u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_9() {
if (bool_regs.y) {
sub_10();
} else {
sub_11();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_10() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_11() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_12();
} else {
sub_13();
}
return false;
}
bool sub_12() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_13() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_14() {
if (bool_regs.y) {
sub_15();
} else {
sub_16();
}
return false;
}
bool sub_15() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_16() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_19() {
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9 = uniforms.f[21];
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (bool_regs.y) {
sub_20();
}
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_20() {
if (uniforms.b[7]) {
sub_21();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_21() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_0() {
sub_1();
sub_19();
return true;
}
// reference: 85FEFB1EB75261E4, 298916613A5C4EB4
// reference: 00310758A279EDE8, B4424FD6FB68AEF8
// program: 298916613A5C4EB4, 58F5ACF9B36D7DF3, B4424FD6FB68AEF8
// shader: 8B31, 465550884350DB6C

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
bool sub_16();
bool sub_2();
bool sub_3();
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
bool sub_0();
bool sub_4();
bool sub_5();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xyz = (vs_in_reg0.xyzz).xyz;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp11.xy = (vs_in_reg1.xyyy).xy;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_16() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = mul_s(vs_in_reg2, reg_tmp13);
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
reg_tmp2.xy = (mul_s(uniforms.f[2].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, reg_tmp2.xyyy)).xy;
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
reg_tmp5.x = (uniforms.f[5].wwww).x;
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_8();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_9();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_10();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
reg_tmp3 = reg_tmp14.xyxy;
reg_tmp3.y = (uniforms.f[0].zzzz + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[0].zzzz + -reg_tmp3.wwww).w;
reg_tmp2 = mul_s(uniforms.f[83].xxyy, reg_tmp3);
reg_tmp2 = -uniforms.f[0].yyyy + reg_tmp2;
reg_tmp2 = mul_s(reg_tmp2, reg_tmp1.xyyx);
reg_tmp2.x = (reg_tmp2.xxxx + -reg_tmp2.yyyy).x;
reg_tmp2.y = (reg_tmp2.zzzz + reg_tmp2.wwww).y;
reg_tmp4.xy = (uniforms.f[3].xyyy).xy;
reg_tmp3.xy = (fma_s(reg_tmp2.xyyy, uniforms.f[3].zwww, reg_tmp4.xyyy)).xy;
reg_tmp3.xy = (uniforms.f[0].yyyy + reg_tmp3.xyyy).xy;
reg_tmp11.xy = (reg_tmp3.xyyy).xy;
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
reg_tmp5.x = (uniforms.f[6].wwww).x;
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_15();
}
reg_tmp3 = reg_tmp14.xyxy;
reg_tmp3.y = (uniforms.f[0].zzzz + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[0].zzzz + -reg_tmp3.wwww).w;
reg_tmp2 = mul_s(uniforms.f[83].zzww, reg_tmp3);
reg_tmp2 = -uniforms.f[0].yyyy + reg_tmp2;
reg_tmp2 = mul_s(reg_tmp2, reg_tmp1.xyyx);
reg_tmp2.x = (reg_tmp2.xxxx + -reg_tmp2.yyyy).x;
reg_tmp2.y = (reg_tmp2.zzzz + reg_tmp2.wwww).y;
reg_tmp4.xy = (uniforms.f[4].xyyy).xy;
reg_tmp3.xy = (fma_s(reg_tmp2.xyyy, uniforms.f[4].zwww, reg_tmp4.xyyy)).xy;
reg_tmp3.xy = (uniforms.f[0].yyyy + reg_tmp3.xyyy).xy;
reg_tmp12.xy = (reg_tmp3.xyyy).xy;
return false;
}
bool sub_8() {
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
return false;
}
bool sub_9() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
return false;
}
bool sub_10() {
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_11() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_12() {
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
return false;
}
bool sub_13() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
return false;
}
bool sub_14() {
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_15() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xyz = (mul_s(uniforms.f[2].xyxx, reg_tmp10.xyzz)).xyz;
reg_tmp0 = uniforms.f[7];
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
reg_tmp3.xyz = (uniforms.f[5].xyzz).xyz;
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
reg_tmp10.xyz = (uniforms.f[5].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_6();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_7();
sub_16();
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
// reference: C53CD4823E885ADC, 465550884350DB6C
// shader: 8B30, 80EE37DD2453A6BA
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
// reference: D4DAD4C669952323, 80EE37DD2453A6BA
// program: 465550884350DB6C, D9AC2D854F4D2F3B, 80EE37DD2453A6BA
// shader: 8B31, 2043B1F9BC437834

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
bool sub_16();
bool sub_2();
bool sub_3();
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
bool sub_16() {
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
reg_tmp5.x = (uniforms.f[5 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_8();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_9();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_10();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
reg_tmp5.x = (uniforms.f[6 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_15();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
bool sub_8() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_9() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_10() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_11() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_12() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_13() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_14() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_15() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
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
sub_16();
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
// reference: 15D7A4B59B2B2298, 2043B1F9BC437834
// program: 2043B1F9BC437834, D9AC2D854F4D2F3B, 80EE37DD2453A6BA
// shader: 8B31, 174AFEF4CB3DDB9E

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
bool sub_19();
bool sub_2();
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
bool sub_19() {
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
reg_tmp5.x = (uniforms.f[5 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
reg_tmp5.x = (uniforms.f[6 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_15();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_16();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_17();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_18();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
bool sub_11() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_12() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_13() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_14() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_15() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_16() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_17() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_18() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
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
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, -uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
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
sub_19();
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
// reference: 5C1EACCA480019B3, 174AFEF4CB3DDB9E
// program: 174AFEF4CB3DDB9E, D9AC2D854F4D2F3B, 80EE37DD2453A6BA
// shader: 8B30, 9A3A24E9FCDCF226
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rrr), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 40F356BC7C161E8C, 9A3A24E9FCDCF226
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 9A3A24E9FCDCF226
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, FC0D8A01C54D4809
// shader: 8B30, DC9190CAFCDCF226
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E21977C161E8C, DC9190CAFCDCF226
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, DC9190CAFCDCF226
// shader: 8B30, A903FF7404272CEA
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
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor1.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (textureProj(tex0, vec3(texcoord0, texcoord0_w)).a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.rgb) + (texcolor2.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
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
// reference: 2EA74BB0F4948734, A903FF7404272CEA
// program: BDB4CD9AD6AFB6D9, EADA2116091C5E01, A903FF7404272CEA
// reference: 602E4019B38771CF, A903FF7404272CEA
// shader: 8B30, 70B01F78B6160B10
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
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
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
// reference: C22C930CC80E49F1, 70B01F78B6160B10
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 70B01F78B6160B10
// shader: 8B30, 61D1CBA4FA0C559B
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
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
vec3 color_output_1 = byteround(clamp((vec3(1) - rounded_primary_color.aaa) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
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
// reference: 072C8BE5A04C6F6A, 61D1CBA4FA0C559B
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 61D1CBA4FA0C559B
// reference: 3A5A347FDCB524B8, B9D0C770DF65157F
// reference: 6AE95EA3DCB524B8, B9D0C770DF65157F
// shader: 8B30, 8CAA8CEE03E0CFE8
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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9B0598519092F778, 8CAA8CEE03E0CFE8
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 8CAA8CEE03E0CFE8
// shader: 8B30, F24210685F812DCE
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
// reference: 349E21975869B3FD, F24210685F812DCE
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, F24210685F812DCE
// reference: 579FE139A04C6F6A, 61D1CBA4FA0C559B
// reference: 8CA598A5C80E49F1, 70B01F78B6160B10
// reference: 3A5A347F1FE36D67, E6D909CE3F49E823
// shader: 8B30, B3943B19E185EBCF
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
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) + (const_color[2].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (texcolor0.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6799AB4E266B003C, B3943B19E185EBCF
// program: A150FC784311F5E1, B6B95AFD9466EC70, B3943B19E185EBCF
// reference: D58C93F89092F778, 8CAA8CEE03E0CFE8
// reference: DC16F279C80E49F1, 70B01F78B6160B10
// reference: 9B059851D7810183, 8CAA8CEE03E0CFE8
// reference: 0DC9D15E4C4BF6D5, D997ADDEB28E7E9E
// shader: 8B30, D9BE516924E97E01
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5A2AB705C8217E36, D9BE516924E97E01
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, D9BE516924E97E01
// shader: 8B30, EBCA3B115080A29D
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
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ED6FCA2BA4B20A8F, EBCA3B115080A29D
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, EBCA3B115080A29D
// shader: 8B30, 2CEC1C47C7448FF0
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7F9FFA20860937B, 2CEC1C47C7448FF0
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 2CEC1C47C7448FF0
// shader: 8B31, 3D6FFAEA80E2F382

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
bool sub_15();
bool sub_2();
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
bool sub_15() {
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
reg_tmp5.x = (uniforms.f[5 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_7();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_8();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_9();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_10();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
reg_tmp5.x = (uniforms.f[6 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
bool sub_7() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_8() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_9() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_10() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_11() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_12() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_13() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_14() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
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
sub_15();
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
// reference: 017B59A184F25752, 3D6FFAEA80E2F382
// program: 3D6FFAEA80E2F382, D9AC2D854F4D2F3B, B8762D381189F738
// reference: 74D33FD6B5F92149, B9D0C770DF65157F
// reference: 2460550A621405DF, E6D909CE3F49E823
// shader: 8B30, 405ED720A6DEE5A9
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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6E8591CA863B523C, 405ED720A6DEE5A9
// program: A150FC784311F5E1, B6B95AFD9466EC70, 405ED720A6DEE5A9
// shader: 8B30, 7E67C25B72AC7D43
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
vec3 color_output_0 = byteround(clamp((texcolor2.rgb) * (texcolor2.aaa) + (texcolor0.rgb) * (vec3(1) - (texcolor2.aaa)), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor2.a) + (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((texcolor1.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (combiner_buffer.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) + (combiner_buffer.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp(min((rounded_primary_color.rgb) + (rounded_primary_color.rgb), vec3(1)) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) + (rounded_primary_color.rgb) - vec3(0.5), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FD7C94AE023C8CC4, 7E67C25B72AC7D43
// program: A150FC784311F5E1, B6B95AFD9466EC70, 7E67C25B72AC7D43
// shader: 8B30, F9D57AA5CF804B4D
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
vec3 color_output_0 = byteround(clamp(min((rounded_primary_color.rgb) + (rounded_primary_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (rounded_primary_color.rgb) - vec3(0.5), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 20C0CA9498EE7DCC, F9D57AA5CF804B4D
// program: A150FC784311F5E1, B6B95AFD9466EC70, F9D57AA5CF804B4D
// shader: 8B30, E1E9F5FB3D1622C4
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
vec3 color_output_0 = byteround(clamp((texcolor1.aaa) * (const_color[0].rgb) + (texcolor1.aaa), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (last_tex_env_out.rgb) + (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (const_color[1].a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rrr) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (const_color[2].a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((combiner_buffer.ggg) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (const_color[3].a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((combiner_buffer.bbb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (const_color[4].a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((texcolor2.rgb) * (const_color[5].rgb) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[5].rgb)), vec3(0), vec3(1)));
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E0D28633BF06448A, E1E9F5FB3D1622C4
// program: B10263C48E967D5C, B6B95AFD9466EC70, E1E9F5FB3D1622C4
// program: 4E7D985E0EC030BF, F777123A8C3D5E20, 64B174ED1C56F246
// program: 465550884350DB6C, D9AC2D854F4D2F3B, B8762D381189F738
// reference: 8B326249D8E9BE55, A90933BCAD7EDACA
// reference: 929FF9D016BF13EE, 2B1D166E58DA6717
// shader: 8B31, 96B032505E07CA7C

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
bool sub_16();
bool sub_3();
bool sub_2();
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
bool sub_16() {
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
reg_tmp5.x = (uniforms.f[5 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_8();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_9();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_10();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
reg_tmp5.x = (uniforms.f[6 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_15();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
bool sub_8() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_9() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_10() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_11() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_12() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_13() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_14() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_15() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
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
sub_16();
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
// reference: A654AA398B73AB41, 96B032505E07CA7C
// program: 96B032505E07CA7C, D9AC2D854F4D2F3B, 80EE37DD2453A6BA
// program: 2043B1F9BC437834, D9AC2D854F4D2F3B, B8762D381189F738
// reference: 7E14216CF4948734, A903FF7404272CEA
// shader: 8B30, B363B3AE3B7973E8
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp(min((last_tex_env_out.rrr) + (last_tex_env_out.ggg), vec3(1)) * (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 29BC0D6A624AE3D3, B363B3AE3B7973E8
// program: A150FC784311F5E1, B6B95AFD9466EC70, B363B3AE3B7973E8
// shader: 8B30, 93B92CB34F0C4944
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp(min((last_tex_env_out.rrr) + (last_tex_env_out.ggg), vec3(1)) * (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F2D6BFEDD6FCCAEF, 93B92CB34F0C4944
// program: A150FC784311F5E1, B6B95AFD9466EC70, 93B92CB34F0C4944
// shader: 8B30, 5C6CC4940B828BD4
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) + (const_color[2].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (texcolor0.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9652CC1788632680, 5C6CC4940B828BD4
// program: A150FC784311F5E1, B6B95AFD9466EC70, 5C6CC4940B828BD4
// shader: 8B30, 4763C3A2C8D60C0E
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) + (const_color[2].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (texcolor0.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (rounded_primary_color.rgb);
float alpha_output_5 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4D387E903CD50FBC, 4763C3A2C8D60C0E
// program: A150FC784311F5E1, B6B95AFD9466EC70, 4763C3A2C8D60C0E
// reference: 3A5A347F58F09B9C, E6D909CE3F49E823
// shader: 8B30, C861C5559374E825
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) + (const_color[2].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = (const_color[4].rgb);
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0B1657281FEA7639, C861C5559374E825
// program: A150FC784311F5E1, B6B95AFD9466EC70, C861C5559374E825
// shader: 8B30, AD52C81D1F44FB3C
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = (const_color[4].rgb);
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ED94EF32C78C5BEB, AD52C81D1F44FB3C
// program: A150FC784311F5E1, B6B95AFD9466EC70, AD52C81D1F44FB3C
// reference: 13F3B02B621405DF, D997ADDEB28E7E9E
// reference: 929FF9D0C80E49F1, 70B01F78B6160B10
// reference: 4340DAF7621405DF, D997ADDEB28E7E9E
// reference: 853FF9249092F778, 8CAA8CEE03E0CFE8
// reference: C22C930C7FF3161F, 2B1D166E58DA6717
// reference: DF4618E64990ED04, 64B174ED1C56F246
// reference: DF4618E60E831BFF, 64B174ED1C56F246
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 9C83F4555B061C91
// shader: 8B30, 37E1BE72D8FA5CEA
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
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
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
// reference: 2012DDDC2D416364, 37E1BE72D8FA5CEA
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 37E1BE72D8FA5CEA
// reference: BB367D84B60358D1, 9483D4BD1E3F4648
// shader: 8B31, 6B7C305F63AF4E6E

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
bool sub_15();
bool sub_2();
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
bool sub_15() {
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
reg_tmp5.x = (uniforms.f[5 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_7();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_8();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_9();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_10();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
reg_tmp5.x = (uniforms.f[6 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
bool sub_7() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_8() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_9() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_10() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_11() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_12() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_13() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_14() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
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
sub_15();
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
// reference: 2BC49DCEAA0789EF, 6B7C305F63AF4E6E
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
// program: 6B7C305F63AF4E6E, D9AC2D854F4D2F3B, 8F3D8BBC3BFBA601
// shader: 8B31, 9889EDC95D31FFC9

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
bool sub_20();
bool sub_2();
bool sub_3();
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
bool sub_20() {
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
reg_tmp5.x = (uniforms.f[5 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_15();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
reg_tmp5.x = (uniforms.f[6 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_16();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_17();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_18();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_19();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
bool sub_12() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_13() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_14() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_15() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_16() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_17() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_18() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_19() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
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
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, -uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
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
sub_20();
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
// reference: 5C1EACCA2BCFF7E0, 9889EDC95D31FFC9
// program: 9889EDC95D31FFC9, D9AC2D854F4D2F3B, 80EE37DD2453A6BA
// shader: 8B30, 39C5F1F70FFDDFF5
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
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0031075856CD1705, 39C5F1F70FFDDFF5
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 39C5F1F70FFDDFF5
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
// reference: 6E8591CA7D9DD952, 468E3AC9726DE092
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 468E3AC9726DE092
// shader: 8B30, AEBDF325C19C6461
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
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E219709F0AE79, AEBDF325C19C6461
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, AEBDF325C19C6461
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
// reference: 003107589088BF8A, 3195BF5BA5FA14CE
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 3195BF5BA5FA14CE
// reference: 4D195E6F0B58002E, E6D909CE3F49E823
// reference: 57CC26BC17E14E80, 78CD8FC0579E0A56
// reference: 2460550A76AF6896, E6D909CE3F49E823
// program: 6B7C305F63AF4E6E, D9AC2D854F4D2F3B, 80EE37DD2453A6BA
// shader: 8B30, 093C239BA7820C55
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
light_vector = normalize(light_src[1].position);
spot_dir = light_src[1].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[1].diffuse * dot_product) + light_src[1].ambient) * 1.0;
specular_sum.rgb += ((light_src[1].specular_0) + (light_src[1].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (primary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
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
// reference: E1338A66DDEB79D6, 093C239BA7820C55
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 093C239BA7820C55
// reference: 884A8103DDEB79D6, 093C239BA7820C55
// program: 96B032505E07CA7C, D9AC2D854F4D2F3B, B8762D381189F738
// shader: 8B31, 83FF1CA4FC50D13D

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
bool sub_15();
bool sub_2();
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
bool sub_15() {
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
reg_tmp5.x = (uniforms.f[5 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_7();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_8();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_9();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_10();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
reg_tmp5.x = (uniforms.f[6 + addr_regs.x].wwww).x;
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
reg_tmp4.xy = (mul_s(reg_tmp14.xxxx, reg_tmp1.xyyy)).xy;
reg_tmp4.x = (fma_s(-reg_tmp14.yyyy, reg_tmp1.yyyy, reg_tmp4.xxxx)).x;
reg_tmp4.y = (fma_s(reg_tmp14.yyyy, reg_tmp1.xxxx, reg_tmp4.yyyy)).y;
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
bool sub_7() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_8() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_9() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_10() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_11() {
reg_tmp14.x = (vs_in_reg0.xxxx).x;
reg_tmp14.y = (vs_in_reg0.yyyy).y;
return false;
}
bool sub_12() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
return false;
}
bool sub_13() {
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
return false;
}
bool sub_14() {
reg_tmp14.x = (-vs_in_reg0.xxxx).x;
reg_tmp14.y = (-vs_in_reg0.yyyy).y;
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
sub_15();
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
// reference: 9E14AD082AF58D7C, 83FF1CA4FC50D13D
// program: 83FF1CA4FC50D13D, D9AC2D854F4D2F3B, B8762D381189F738
// reference: E1338A66B4A77C27, 093C239BA7820C55
// reference: D6A06F47634A58B1, 5FF2DB80DA8B41EA
// shader: 8B30, E33B96DEBFF72D67
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
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) + (const_color[2].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = (const_color[4].rgb);
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 21B782F6055479B9, E33B96DEBFF72D67
// program: A150FC784311F5E1, B6B95AFD9466EC70, E33B96DEBFF72D67
// reference: 982964EE4D15ABBB, 5FF2DB80DA8B41EA
// shader: 8B30, 9462DC730E34FE91
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
vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (const_color[5].a);
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
// reference: 3F702F22E31AC53A, 9462DC730E34FE91
// program: 0000000000000000, 0000000000000000, 9462DC730E34FE91
// shader: 8B30, 7745C85789E86894
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
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((texcolor0.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = (const_color[5].a);
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
// reference: FE517898737D3F36, 7745C85789E86894
// program: 0000000000000000, 0000000000000000, 7745C85789E86894
// reference: AEE21244737D3F36, 7745C85789E86894
// reference: B8D5EF3BB9AEB1B9, 7745C85789E86894
// reference: B0D87331346EC9CD, 7745C85789E86894
// program: 4E7D985E0EC030BF, F777123A8C3D5E20, A90933BCAD7EDACA
// shader: 8B31, 8D65E0DFADA6CD13

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
bool sub_15();
bool sub_2();
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
bool sub_0();
bool sub_3();
bool sub_4();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xyz = (vs_in_reg0.xyzz).xyz;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp11.xy = (vs_in_reg1.xyyy).xy;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_15() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = mul_s(vs_in_reg2, reg_tmp13);
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
reg_tmp2.xy = (mul_s(uniforms.f[2].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, reg_tmp2.xyyy)).xy;
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
reg_tmp5.x = (uniforms.f[5].wwww).x;
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_7();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_8();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_9();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_10();
}
reg_tmp3 = reg_tmp14.xyxy;
reg_tmp3.y = (uniforms.f[0].zzzz + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[0].zzzz + -reg_tmp3.wwww).w;
reg_tmp2 = mul_s(uniforms.f[83].xxyy, reg_tmp3);
reg_tmp2 = -uniforms.f[0].yyyy + reg_tmp2;
reg_tmp2 = mul_s(reg_tmp2, reg_tmp1.xyyx);
reg_tmp2.x = (reg_tmp2.xxxx + -reg_tmp2.yyyy).x;
reg_tmp2.y = (reg_tmp2.zzzz + reg_tmp2.wwww).y;
reg_tmp4.xy = (uniforms.f[3].xyyy).xy;
reg_tmp3.xy = (fma_s(reg_tmp2.xyyy, uniforms.f[3].zwww, reg_tmp4.xyyy)).xy;
reg_tmp3.xy = (uniforms.f[0].yyyy + reg_tmp3.xyyy).xy;
reg_tmp11.xy = (reg_tmp3.xyyy).xy;
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
reg_tmp5.x = (uniforms.f[6].wwww).x;
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
reg_tmp3 = reg_tmp14.xyxy;
reg_tmp3.y = (uniforms.f[0].zzzz + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[0].zzzz + -reg_tmp3.wwww).w;
reg_tmp2 = mul_s(uniforms.f[83].zzww, reg_tmp3);
reg_tmp2 = -uniforms.f[0].yyyy + reg_tmp2;
reg_tmp2 = mul_s(reg_tmp2, reg_tmp1.xyyx);
reg_tmp2.x = (reg_tmp2.xxxx + -reg_tmp2.yyyy).x;
reg_tmp2.y = (reg_tmp2.zzzz + reg_tmp2.wwww).y;
reg_tmp4.xy = (uniforms.f[4].xyyy).xy;
reg_tmp3.xy = (fma_s(reg_tmp2.xyyy, uniforms.f[4].zwww, reg_tmp4.xyyy)).xy;
reg_tmp3.xy = (uniforms.f[0].yyyy + reg_tmp3.xyyy).xy;
reg_tmp12.xy = (reg_tmp3.xyyy).xy;
return false;
}
bool sub_7() {
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
return false;
}
bool sub_8() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
return false;
}
bool sub_9() {
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_10() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_11() {
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
return false;
}
bool sub_12() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
return false;
}
bool sub_13() {
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_14() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xyz = (mul_s(uniforms.f[2].xyxx, reg_tmp10.xyzz)).xyz;
reg_tmp2 = reg_tmp10.xzyw;
reg_tmp2.z = (-reg_tmp2.zzzz).z;
reg_tmp14 = reg_tmp2;
reg_tmp0 = uniforms.f[7];
sub_2();
reg_tmp2.x = dot_s(reg_tmp14, reg_tmp6);
reg_tmp2.y = dot_s(reg_tmp14, reg_tmp7);
reg_tmp2.z = dot_s(reg_tmp14, reg_tmp8);
reg_tmp2.w = dot_s(reg_tmp14, reg_tmp9);
reg_tmp6 = uniforms.f[8];
reg_tmp7 = uniforms.f[9];
reg_tmp8 = uniforms.f[10];
reg_tmp9 = uniforms.f[0].xxxz;
reg_tmp6.w = (uniforms.f[5].xxxx).w;
reg_tmp7.w = (uniforms.f[5].yyyy).w;
reg_tmp8.w = (uniforms.f[5].zzzz).w;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp3.xyz = (uniforms.f[5].xyzz).xyz;
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
reg_tmp2 = uniforms.f[1];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_6();
sub_15();
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
// reference: 49AB51BE5BD85006, 8D65E0DFADA6CD13
// program: 8D65E0DFADA6CD13, D9AC2D854F4D2F3B, B8762D381189F738
// reference: 349E2197BFC60E39, 4A60600E5756925D
// program: 3A1D4E07C99F24D3, C8DF8CC87A4E8CE6, 4A60600E5756925D
// shader: 8B31, 1CC9D11ADCEA4CA3

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
bool sub_19();
bool sub_2();
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
reg_tmp10 = uniforms.f[0].xxxz;
reg_tmp10.xyz = (vs_in_reg0.xyzz).xyz;
reg_tmp11 = uniforms.f[0].xxxx;
reg_tmp12 = uniforms.f[0].xxxx;
reg_tmp11.xy = (vs_in_reg1.xyyy).xy;
reg_tmp13 = uniforms.f[0].zzzz;
reg_tmp4 = uniforms.f[0].xxxx;
reg_tmp5 = uniforms.f[0].xxxx;
return false;
}
bool sub_19() {
vs_out_attr0 = reg_tmp10;
reg_tmp13 = mul_s(uniforms.f[82], reg_tmp13);
vs_out_attr1 = mul_s(vs_in_reg2, reg_tmp13);
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
reg_tmp2.xy = (mul_s(uniforms.f[2].zwww, reg_tmp2.xxxx)).xy;
reg_tmp2.xy = (uniforms.f[0].yyyy + reg_tmp2.xyyy).xy;
reg_tmp3.xy = (floor(reg_tmp2.xyyy)).xy;
reg_tmp2.xy = (mul_s(uniforms.f[81].yyyy, reg_tmp3.xyyy)).xy;
reg_tmp2.xy = (uniforms.f[2].zwww + -reg_tmp2.xyyy).xy;
reg_tmp0.xy = (min_s(uniforms.f[81].xxxx, reg_tmp2.xyyy)).xy;
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
reg_tmp5.x = (uniforms.f[5].wwww).x;
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_11();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_12();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_13();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_14();
}
reg_tmp3 = reg_tmp14.xyxy;
reg_tmp3.y = (uniforms.f[0].zzzz + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[0].zzzz + -reg_tmp3.wwww).w;
reg_tmp2 = mul_s(uniforms.f[83].xxyy, reg_tmp3);
reg_tmp2 = -uniforms.f[0].yyyy + reg_tmp2;
reg_tmp2 = mul_s(reg_tmp2, reg_tmp1.xyyx);
reg_tmp2.x = (reg_tmp2.xxxx + -reg_tmp2.yyyy).x;
reg_tmp2.y = (reg_tmp2.zzzz + reg_tmp2.wwww).y;
reg_tmp4.xy = (uniforms.f[3].xyyy).xy;
reg_tmp3.xy = (fma_s(reg_tmp2.xyyy, uniforms.f[3].zwww, reg_tmp4.xyyy)).xy;
reg_tmp3.xy = (uniforms.f[0].yyyy + reg_tmp3.xyyy).xy;
reg_tmp11.xy = (reg_tmp3.xyyy).xy;
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
reg_tmp5.x = (uniforms.f[6].wwww).x;
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
bool_regs.x = uniforms.f[0].xxxx.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].xxxx.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_15();
}
bool_regs.x = uniforms.f[0].zzzz.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].zzzz.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_16();
}
bool_regs.x = uniforms.f[0].wwww.x == reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_17();
}
bool_regs.x = uniforms.f[0].wwww.x < reg_tmp5.xxxx.x;
bool_regs.y = uniforms.f[0].wwww.y != reg_tmp5.xxxx.y;
if (bool_regs.x) {
sub_18();
}
reg_tmp3 = reg_tmp14.xyxy;
reg_tmp3.y = (uniforms.f[0].zzzz + -reg_tmp3.yyyy).y;
reg_tmp3.w = (uniforms.f[0].zzzz + -reg_tmp3.wwww).w;
reg_tmp2 = mul_s(uniforms.f[83].zzww, reg_tmp3);
reg_tmp2 = -uniforms.f[0].yyyy + reg_tmp2;
reg_tmp2 = mul_s(reg_tmp2, reg_tmp1.xyyx);
reg_tmp2.x = (reg_tmp2.xxxx + -reg_tmp2.yyyy).x;
reg_tmp2.y = (reg_tmp2.zzzz + reg_tmp2.wwww).y;
reg_tmp4.xy = (uniforms.f[4].xyyy).xy;
reg_tmp3.xy = (fma_s(reg_tmp2.xyyy, uniforms.f[4].zwww, reg_tmp4.xyyy)).xy;
reg_tmp3.xy = (uniforms.f[0].yyyy + reg_tmp3.xyyy).xy;
reg_tmp12.xy = (reg_tmp3.xyyy).xy;
return false;
}
bool sub_11() {
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
return false;
}
bool sub_12() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
return false;
}
bool sub_13() {
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_14() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_15() {
reg_tmp14.xy = (vs_in_reg1.xyyy).xy;
return false;
}
bool sub_16() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
return false;
}
bool sub_17() {
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_18() {
reg_tmp14.x = (uniforms.f[0].zzzz + -vs_in_reg1.xxxx).x;
reg_tmp14.y = (uniforms.f[0].zzzz + -vs_in_reg1.yyyy).y;
return false;
}
bool sub_0() {
sub_1();
reg_tmp10.xy = (uniforms.f[84].xyyy + reg_tmp10.xyyy).xy;
reg_tmp10.xyz = (mul_s(uniforms.f[2].xyxx, reg_tmp10.xyzz)).xyz;
reg_tmp14 = reg_tmp10;
reg_tmp0 = uniforms.f[7];
sub_2();
reg_tmp10.x = dot_s(reg_tmp14, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp14, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp14, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp14, reg_tmp9);
reg_tmp2.x = (-uniforms.f[8].yyyy).x;
reg_tmp2.y = (-uniforms.f[9].yyyy).y;
reg_tmp2.z = (-uniforms.f[10].yyyy).z;
reg_tmp3.xyz = (uniforms.f[6].xyzz).xyz;
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
reg_tmp5.xyz = (mul_s(reg_tmp4.yzxx, reg_tmp3.zxyy)).xyz;
reg_tmp5.xyz = (fma_s(-reg_tmp3.yzxx, reg_tmp4.zxyy, reg_tmp5)).xyz;
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
reg_tmp3.xyz = (uniforms.f[5].xyzz).xyz;
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
reg_tmp10.xyz = (uniforms.f[5].xyzz + reg_tmp2.xyzz).xyz;
reg_tmp10.xyz = (fma_s(reg_tmp3.xyzz, uniforms.f[84].zzzz, reg_tmp10.xyzz)).xyz;
sub_9();
reg_tmp2 = reg_tmp10;
reg_tmp10.x = dot_s(reg_tmp2, reg_tmp6);
reg_tmp10.y = dot_s(reg_tmp2, reg_tmp7);
reg_tmp10.z = dot_s(reg_tmp2, reg_tmp8);
reg_tmp10.w = dot_s(reg_tmp2, reg_tmp9);
reg_tmp2 = uniforms.f[1];
reg_tmp3 = max_s(uniforms.f[0].xxxx, reg_tmp2);
reg_tmp2 = min_s(uniforms.f[0].zzzz, reg_tmp3);
reg_tmp13 = reg_tmp2;
sub_10();
sub_19();
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
// reference: 6F26EA1432327EA1, 1CC9D11ADCEA4CA3
// program: 1CC9D11ADCEA4CA3, D9AC2D854F4D2F3B, 80EE37DD2453A6BA
// reference: 929FF9D07FF3161F, 2B1D166E58DA6717
// shader: 8B30, 526BD91D234E640A
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
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = (const_color[4].rgb);
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ADB427D9B9534B58, 526BD91D234E640A
// program: A150FC784311F5E1, B6B95AFD9466EC70, 526BD91D234E640A
// reference: 972873FD737D3F36, 7745C85789E86894
// shader: 8B30, 8BCEDB643A989774
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
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E2197335E535C, 8BCEDB643A989774
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 8BCEDB643A989774
// shader: 8B30, A0867826DF20D235
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
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 00310758034D5CD2, A0867826DF20D235
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, A0867826DF20D235
// shader: 8B30, 3FDA6338E0440C6C
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7F9FFA269DA6DA9, 3FDA6338E0440C6C
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 3FDA6338E0440C6C
// shader: 8B30, 850A2A910B08E069
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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 349E2197EFD3FDD4, 850A2A910B08E069
// program: 6B4C12038C932EEC, C8DF8CC87A4E8CE6, 850A2A910B08E069
// shader: 8B30, F0C783F3F5A9D4CD
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
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) + (const_color[2].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 1.0, alpha_output_2 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (texcolor0.rgb) * (vec3(1) - (last_tex_env_out.aaa)), vec3(0), vec3(1)));
float alpha_output_4 = (texcolor0.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb) + (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7AA24EB192DD2900, F0C783F3F5A9D4CD
// program: A150FC784311F5E1, B6B95AFD9466EC70, F0C783F3F5A9D4CD

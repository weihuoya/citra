vec2 norm2denorm(in vec2 uv, in ivec2 texture_size)
{
    return uv * vec2(texture_size) - 0.5;
}
ivec2 denorm2idx(in vec2 d_uv)
{
    return ivec2(floor(d_uv));
}
ivec2 norm2idx(in vec2 uv, in ivec2 texture_size)
{
    return denorm2idx(norm2denorm(uv, texture_size));
}
vec2 idx2norm(in ivec2 idx, in ivec2 texture_size)
{
    vec2 denorm_uv = vec2(idx) + 0.5;
    return denorm_uv / vec2(texture_size);
}
vec4 texel_fetch(in ivec2 idx, in ivec2 texture_size)
{
    vec2 uv = idx2norm(idx, texture_size);
    return SampleLocation(uv);
}
vec4 hybridFilter(in vec2 uv)
{
    ivec2 texSize = SampleSize();
    vec2 denorm_uv = norm2denorm(uv,texSize);
    ivec2 idx_low = denorm2idx(denorm_uv);
    vec2 ratio = denorm_uv - vec2(idx_low);
    ivec2 rounded_idx = idx_low + ivec2(step(0.5, ratio));

    ivec2 idx00 = idx_low;
    vec4 t00 = texel_fetch(idx00, texSize);

    ivec2 idx01 = idx00 + ivec2(0, 1);
    vec4 t01 = texel_fetch(idx01, texSize);

    ivec2 idx10 = idx00 + ivec2(1, 0);
    vec4 t10 = texel_fetch(idx10, texSize);

    ivec2 idx11 = idx00 + ivec2(1, 1);
    vec4 t11 = texel_fetch(idx11, texSize);

    /*
    * radius is the distance from the edge where interpolation happens.
    * It's calculated based on  how big a fragment is in denormalized
    * texture coordinates.
    *
    * E.g.: If a texel maps to 5 fragments, then each fragment is
    * 1/5 texels big. So the smooth transition should be between one and
    * two fragments big, since there are enough fragments to show the full
    * color of the texel.
    *
    * If a fragment is larger than one texel, we don't care, we're already
    * sampling the wrong texels, and should be using mipmaps instead.
    */

    // Here, fwidth() is used to estimte how much denorm_uv changes per fragment.
    // But we divide it by 2, since fwidth() is adding abs(dx) + abs(dy).
    vec2 fragment_size = fwidth(denorm_uv) / 2.0;

    float is_frag_gt1, radius;
    // Do nothing if fragment is greater than 1 texel
    // Don't make the transition more than one fragment (+/- 0.5 fragment)
    is_frag_gt1 = step(1.0, fragment_size.s);
    radius = min(fragment_size.s, 0.5);
    ratio.s = ratio.s * is_frag_gt1 + smoothstep(0.5 - radius, 0.5 + radius, ratio.s) * (1.0 - is_frag_gt1);
    is_frag_gt1 = step(1.0, fragment_size.t);
    radius = min(fragment_size.t, 0.5);
    ratio.t = ratio.t * is_frag_gt1 + smoothstep(0.5 - radius, 0.5 + radius, ratio.t) * (1.0 - is_frag_gt1);

    // interpolate first on S direction then on T.
    vec4 top = mix(t00, t10, ratio.s);
    vec4 bottom = mix(t01, t11, ratio.s);
    return mix(top, bottom, ratio.t);
}

void main()
{
    SetOutput(hybridFilter(GetCoordinates()));
}

//! mag_filter = nearest
//! min_filter = nearest

float bloom(vec3 color)
{
  float shine = 0.05;
  vec3 gray_coeff = vec3(0.30, 0.59, 0.11);
  float bright = dot(color, gray_coeff);
  return mix(1.0 + shine, 1.0 - shine, bright);
}

vec3 lookup(vec2 base)
{
  vec3 color = Sample().rgb;
  float delta = distance(base, vec2(0.25));
  return color * exp(-delta * bloom(color)) * 1.4;
}

void main()
{
  vec2 uv = GetCoordinates();
  vec2 base = fract(uv * GetOnScreenSize() / 1.2);
  vec3 color = lookup(base);
  SetOutput(vec4(color, 1));
}
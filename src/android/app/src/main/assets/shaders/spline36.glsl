float2 u_texelDelta;

float spline36_0_1(float x) {
  return ((13.0 / 11.0 * x - 453.0 / 209.0) * x - 3.0 / 209.0) * x + 1.0;
}

float spline36_1_2(float x) {
  return ((-6.0 / 11.0 * x + 612.0 / 209.0) * x - 1038.0 / 209.0) * x + 540.0 / 209.0;
}

float spline36_2_3(float x) {
  return ((1.0 / 11.0 * x - 159.0 / 209.0) * x + 434.0 / 209.0) * x - 384.0 / 209.0;
}

float4 rgb(int inputX, int inputY) {
  return SampleLocation((float2(inputX, inputY) + float2(0.5, 0.5)) * u_texelDelta);
}

float4 interpolateHorizontally(float2 inputPos, int2 inputPosFloor, int dy) {
  float sumOfWeights = 0.0;
  float4 sumOfWeightedPixel = float4(0.0);

  float x;
  float weight;

  x = inputPos.x - float(inputPosFloor.x - 2);
  weight = spline36_2_3(x);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * rgb(inputPosFloor.x - 2, inputPosFloor.y + dy);

  --x;
  weight = spline36_1_2(x);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * rgb(inputPosFloor.x - 1, inputPosFloor.y + dy);

  --x;
  weight = spline36_0_1(x);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * rgb(inputPosFloor.x + 0, inputPosFloor.y + dy);

  x = 1.0 - x;
  weight = spline36_0_1(x);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * rgb(inputPosFloor.x + 1, inputPosFloor.y + dy);

  ++x;
  weight = spline36_1_2(x);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * rgb(inputPosFloor.x + 2, inputPosFloor.y + dy);

  ++x;
  weight = spline36_2_3(x);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * rgb(inputPosFloor.x + 3, inputPosFloor.y + dy);

  return sumOfWeightedPixel / sumOfWeights;
}

float4 process(float2 outputPos) {
  float2 inputPos = outputPos / u_texelDelta;
  int2 inputPosFloor = int2(inputPos);

  // Vertical interporation
  float sumOfWeights = 0.0;
  float4 sumOfWeightedPixel = float4(0.0);

  float weight;
  float y;

  y = inputPos.y - float(inputPosFloor.y - 2);
  weight = spline36_2_3(y);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * interpolateHorizontally(inputPos, inputPosFloor, -2);

  --y;
  weight = spline36_1_2(y);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * interpolateHorizontally(inputPos, inputPosFloor, -1);

  --y;
  weight = spline36_0_1(y);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * interpolateHorizontally(inputPos, inputPosFloor, +0);

  y = 1.0 - y;
  weight = spline36_0_1(y);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * interpolateHorizontally(inputPos, inputPosFloor, +1);

  ++y;
  weight = spline36_1_2(y);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * interpolateHorizontally(inputPos, inputPosFloor, +2);

  ++y;
  weight = spline36_2_3(y);
  sumOfWeights += weight;
  sumOfWeightedPixel += weight * interpolateHorizontally(inputPos, inputPosFloor, +3);

  return float4((sumOfWeightedPixel / sumOfWeights).xyz, 1.0);
}

void main()
{
  u_texelDelta = float2(1.0f) / float2(SampleSize());
  SetOutput(process(GetCoordinates()));
}

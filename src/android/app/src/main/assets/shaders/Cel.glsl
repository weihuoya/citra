#define WhtCutoff 0.97
#define BlkCutoff 0.03
#define ShdLevels 9.0
#define SatModify 1.15
#define OtlModify 0.20
#define ShdWeight 0.50

float3 RGB2HSL(float3 cRGB)
{
    float cR = cRGB[0], cG = cRGB[1], cB = cRGB[2];
    float vMin = min(min(cR, cG), cB), vMax = max(max(cR, cG), cB);
    float dMax = vMax - vMin, vS = 0.0, vH = 0.0, vL = (vMax + vMin) / 2.0;

    // gray, no chroma
    if(dMax == 0.0) {
      vH = 0.0; vS = vH;

    // chromatic data
    } else {
        if(vL < 0.5) { vS = dMax / (vMax + vMin); }
        else         { vS = dMax / (2.0 - vMax - vMin); }

        float dR = (((vMax - cR) * 0.1667) + (dMax * 0.5)) / dMax;
        float dG = (((vMax - cG) * 0.1667) + (dMax * 0.5)) / dMax;
        float dB = (((vMax - cB) * 0.1667) + (dMax * 0.5)) / dMax;

        if     (cR >= vMax) { vH = dB - dG; }
        else if(cG >= vMax) { vH = 0.3333 + dR - dB; }
        else if(cB >= vMax) { vH = 0.6667 + dG - dR; }

        if     (vH < 0.0) { vH += 1.0; }
        else if(vH > 1.0) { vH -= 1.0; }
    }
    return float3(vH, vS, vL);
}

float Hue2RGB(float v1, float v2, float vH)
{
    float v3 = 0.0;

    if     (vH < 0.0) { vH += 1.0; }
    else if(vH > 1.0) { vH -= 1.0; }

    if     ((6.0 * vH) < 1.0) { v3 = v1 + (v2 - v1) * 6.0 * vH; }
    else if((2.0 * vH) < 1.0) { v3 = v2; }
    else if((3.0 * vH) < 2.0) { v3 = v1 + (v2 - v1) * (0.6667 - vH) * 6.0; }
    else                      { v3 = v1; }

    return v3;
}

float3 HSL2RGB(float3 vHSL)
{
    float cR = 0.0, cG = cR, cB = cR;

    if(vHSL[1] == 0.0) {
      cR = vHSL[2], cG = cR, cB = cR;

    } else {
        float v1 = 0.0, v2 = v1;

        if(vHSL[2] < 0.5) { v2 = vHSL[2] * (1.0 + vHSL[1] ); }
        else              { v2 = (vHSL[2] + vHSL[1] ) - (vHSL[1] * vHSL[2] ); }

        v1 = 2.0 * vHSL[2] - v2;

        cR = Hue2RGB(v1, v2, vHSL[0] + 0.3333);
        cG = Hue2RGB(v1, v2, vHSL[0] );
        cB = Hue2RGB(v1, v2, vHSL[0] - 0.3333);
    }
    return float3(cR, cG, cB);
}

float3 colorAdjust(float3 cRGB)
{
    float3 cHSL = RGB2HSL(cRGB);

    float cr = 1.0 / ShdLevels;

    // brightness modifier
    float BrtModify = mod(cHSL[2], cr);

    if     (cHSL[2] > WhtCutoff) { cHSL[1]  = 1.0; cHSL[2] = 1.0; }
    else if(cHSL[2] > BlkCutoff) { cHSL[1] *= SatModify; cHSL[2] += (cHSL[2] * cr - BrtModify); }
    else                         { cHSL[1]  = 0.0; cHSL[2] = 0.0; }
    cRGB = 1.2 * HSL2RGB(cHSL);

    return cRGB;
}

float4 CelShader(float2 invSize, float2 TexCoord)
{
    float4 offset;
    float4 TEX0 = TexCoord.xyxy;

    offset.xy = -(offset.zw = float2(invSize.x, 0.0));
    float4 TEX1 = TEX0 + offset;

    offset.xy = -(offset.zw = float2(0.0, invSize.y));
    float4 TEX2 = TEX0 + offset;
    float4 TEX3 = TEX1 + offset;

    float3 c0 = SampleLocation(TEX3.xy).rgb;
    float3 c1 = SampleLocation(TEX2.xy).rgb;
    float3 c2 = SampleLocation(TEX3.zy).rgb;
    float3 c3 = SampleLocation(TEX1.xy).rgb;
    float3 c4 = SampleLocation(TEX0.xy).rgb;
    float3 c5 = SampleLocation(TEX1.zw).rgb;
    float3 c6 = SampleLocation(TEX3.xw).rgb;
    float3 c7 = SampleLocation(TEX2.zw).rgb;
    float3 c8 = SampleLocation(TEX3.zw).rgb;

    float3 c9 = ((c0 + c2 + c6 + c8) * 0.15 + (c1 + c3 + c5 + c7) * 0.25 + c4) / 2.6;

    float3 o = float3(1.0); float3 h = float3(0.05); float3 hz = h; float k = 0.005;
    float kz = 0.007; float i = 0.0;

    float3 cz = (c9 + h) / (dot(o, c9) + k);

    hz = (cz - ((c0 + h) / (dot(o, c0) + k))); i  = kz / (dot(hz, hz) + kz);
    hz = (cz - ((c1 + h) / (dot(o, c1) + k))); i += kz / (dot(hz, hz) + kz);
    hz = (cz - ((c2 + h) / (dot(o, c2) + k))); i += kz / (dot(hz, hz) + kz);
    hz = (cz - ((c3 + h) / (dot(o, c3) + k))); i += kz / (dot(hz, hz) + kz);
    hz = (cz - ((c5 + h) / (dot(o, c5) + k))); i += kz / (dot(hz, hz) + kz);
    hz = (cz - ((c6 + h) / (dot(o, c6) + k))); i += kz / (dot(hz, hz) + kz);
    hz = (cz - ((c7 + h) / (dot(o, c7) + k))); i += kz / (dot(hz, hz) + kz);
    hz = (cz - ((c8 + h) / (dot(o, c8) + k))); i += kz / (dot(hz, hz) + kz);

    i /= 8.0; i = pow(i, 0.75);

    if(i < OtlModify) { i = OtlModify; }
    c9 = min(o, min(c9, c9 + dot(o, c9)));
    return vec4(mix(c4 * i, colorAdjust(c9 * i), ShdWeight), 1.0);
}

void main()
{
    SetOutput(CelShader( 1.0 / GetResolution(), GetCoordinates()));
}

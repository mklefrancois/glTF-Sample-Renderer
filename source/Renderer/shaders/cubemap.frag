precision highp float;


#include <tonemapping.glsl>


uniform float u_EnvIntensity;
uniform float u_EnvBlurNormalized;
uniform int u_MipCount;
uniform samplerCube u_GGXEnvSampler;

layout(location = 0) out vec4 FragColor;
layout(location = 1) out uint toneMapFlag;
in vec3 v_TexCoords;


void main()
{
    vec4 color = textureLod(u_GGXEnvSampler, v_TexCoords, u_EnvBlurNormalized * float(u_MipCount - 1));
    color.rgb *= u_EnvIntensity;
    color.a = 1.0;
    FragColor = color.rgba;
    toneMapFlag = 2u;
}

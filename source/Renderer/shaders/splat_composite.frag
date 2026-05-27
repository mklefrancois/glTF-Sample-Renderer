precision highp float;

#include <tonemapping.glsl>

// Composites the splat isolation framebuffer onto the main framebuffer.
// Outputs:
//   location 0 – color (blended onto main)
//   location 1 – toneMapFlag  1u = "linearToSRGB" for the tonemap_main pass

uniform sampler2D u_SplatSampler;
uniform bool      u_SrgbToLinear;

in vec2 v_uv;

layout(location = 0) out vec4 g_finalColor;
layout(location = 1) out uint g_toneMapFlag;

void main() {
    vec4 splat = texture(u_SplatSampler, v_uv);

    // Discard fully-transparent fragments so the toneMapFlag texture is not
    // overwritten for pixels that were never touched by a splat.
#ifndef TONEMAP_NONE
    vec3 multiplied = splat.rgb * splat.a;
    float maxComponent = max(max(multiplied.r, multiplied.g), multiplied.b);
    if (maxComponent < 1.0 / 255.0) {
        discard;
    } 
#endif
    splat.rgb = clamp(splat.rgb, vec3(0.0), vec3(1.0));
    if (u_SrgbToLinear) {
        splat.rgb  = sRGBToLinear(splat.rgb);
    }
    g_finalColor  = splat;
    // The composited content is linear; tonemap_main.frag should apply
    // linearToSRGB (flag value 0) in its final step.
    g_toneMapFlag = 1u;
    
}

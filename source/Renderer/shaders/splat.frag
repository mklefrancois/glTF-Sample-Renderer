precision highp float;

in vec4 v_color;  // (r, g, b, opacity)
in vec2 v_uv;     // pixel-space offset from splat centre
in vec3 v_conic;  // (A, B, C) coefficients of the conic section defining the splat's shape

layout(location = 0) out vec4 g_finalColor;

void main() {

    // Equation 4
    float exponent = -0.5 * (v_conic.x * v_uv.x * v_uv.x + v_conic.z * v_uv.y * v_uv.y) - v_conic.y * v_uv.x * v_uv.y;

    if (exponent > 0.0) {
        discard;
    }

    float alpha = min(0.99, exp(exponent) * v_color.a); // opacity modulated by Gaussian falloff
    if (alpha < 1.0 / 255.0) {
        discard;
    }
    g_finalColor = max(vec4(v_color.rgb * alpha, alpha), vec4(0.0)); // premultiplied-alpha output
}

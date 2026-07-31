precision highp float;

// Fullscreen quad vertex shader.
// Expects the splatVBO quad: positions in [-1, 1] x [-1, 1].
in vec2 a_position;

out vec2 v_uv;

void main()
{
    v_uv = a_position * 0.5 + 0.5;
    gl_Position = vec4(a_position, 0.0, 1.0);
}

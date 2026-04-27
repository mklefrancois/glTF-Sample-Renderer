// Original CUDA kernel https://github.com/graphdeco-inria/diff-gaussian-rasterization/blob/9c5c2028f6fbee2be239bc4c9421ff894fe4fbe0/cuda_rasterizer/forward.cu

precision highp float;
precision highp int;

in vec2 a_position;
in uint a_instance_sort_index;

out vec4 v_color;
out vec2 v_uv;
out vec3 v_conic;

uniform mat4 u_ProjectionMatrix;
uniform mat4 u_ViewMatrix;
uniform mat4 u_ModelMatrix;
uniform uint u_TextureWidth;
uniform ivec2 u_FramebufferSize;
uniform vec2 u_FocalLength;

#ifdef POSITION_IS_INTEGER
uniform isampler2D u_POSITIONSampler;
#elif defined(POSITION_IS_UINTEGER)
uniform usampler2D u_POSITIONSampler;
#else
uniform sampler2D u_POSITIONSampler;
#endif

#ifdef ROTATION_IS_INTEGER
uniform isampler2D u_ROTATIONSampler;
#else
uniform sampler2D u_ROTATIONSampler;
#endif

#ifdef SCALE_IS_UINTEGER
uniform usampler2D u_SCALESampler;
#else
uniform sampler2D u_SCALESampler;
#endif

#ifdef OPACITY_IS_UINTEGER
uniform usampler2D u_OPACITYSampler;
#else
uniform sampler2D u_OPACITYSampler;
#endif

uniform highp sampler2DArray u_SHCoefficientsSampler;

mat3 computeC(vec3 scale, vec4 rotation)
{

    float qx = rotation.x;
    float qy = rotation.y;
    float qz = rotation.z;
    float qw = rotation.w;
    float sx = scale.x;
    float sy = scale.y;
    float sz = scale.z;

    mat3 C = mat3(
        sx * (1.0 - 2.0*(qy*qy + qz*qz)),  sx * (2.0*(qx*qy + qw*qz)),        sx * (2.0*(qx*qz - qw*qy)),
        sy * (2.0*(qx*qy - qw*qz)),        sy * (1.0 - 2.0*(qx*qx + qz*qz)),  sy * (2.0*(qy*qz + qw*qx)),
        sz * (2.0*(qx*qz + qw*qy)),        sz * (2.0*(qy*qz - qw*qx)),        sz * (1.0 - 2.0*(qx*qx + qy*qy))
    );
 
    return C;
}

vec3 computeCameraCovariance(mat3 world_covariance, vec3 view_splat_center)
{
    float x = view_splat_center.x;
    float y = view_splat_center.y;
    float z = view_splat_center.z;

    mat3 J = mat3(
        u_FocalLength.x / z, 0.0, -(u_FocalLength.x * x) / (z * z),
		0.0, u_FocalLength.y / z, -(u_FocalLength.y * y) / (z * z),
		0, 0, 0
    );

    mat3 W = transpose(mat3(u_ViewMatrix));

    // Equation 5
    mat3 T = W * J;

    mat3 cov = transpose(T) * transpose(world_covariance) * T;
    
    // Low-pass filter (EWA Splatting)
    cov[0][0] += 0.3;
    cov[1][1] += 0.3;

    return vec3(cov[0][0], cov[0][1], cov[1][1]);
}

vec3 calculateSphericalHarmonics(ivec2 texelCoord)
{
    ivec3 coord = ivec3(texelCoord.x, texelCoord.y, 0);
    // Degree 0
    vec3 sh0 = texelFetch(u_SHCoefficientsSampler, coord, 0).rgb;

    vec3 result = sh0 * 0.2820947917738781;

    //TODO

    result += 0.5;
    return result;

}


void main()
{
    vec3 splat_center;
    ivec2 texelCoord = ivec2(a_instance_sort_index % u_TextureWidth, a_instance_sort_index / u_TextureWidth);
#if defined(POSITION_IS_INTEGER) || defined(POSITION_IS_UINTEGER)
    splat_center = vec3(texelFetch(u_POSITIONSampler, texelCoord, 0).xyz);
#ifdef POSITION_NEEDS_NORMALIZATION
#ifdef POSITION_IS_UINTEGER
    splat_center = splat_center / 65535.0; // normalize unsigned short
#else
    splat_center = max(splat_center / 32767.0, -1.0); // normalize signed short
#endif
#endif
#else
    splat_center = texelFetch(u_POSITIONSampler, texelCoord, 0).xyz;
#endif

    vec4 rotation;
#if defined(ROTATION_IS_INTEGER)
    rotation = vec4(texelFetch(u_ROTATIONSampler, texelCoord, 0).xyzw);
#ifdef ROTATION_NEEDS_NORMALIZATION
    rotation = max(rotation / 32767.0, -1.0); // normalize signed short
#endif
#else
    rotation = texelFetch(u_ROTATIONSampler, texelCoord, 0);
#endif

    vec3 scale;
#if defined(SCALE_IS_UINTEGER)
    scale = vec3(texelFetch(u_SCALESampler, texelCoord, 0).xyz);
#ifdef SCALE_NEEDS_NORMALIZATION
    scale = scale / 65535.0; // normalize unsigned short
#endif
#else
    scale = texelFetch(u_SCALESampler, texelCoord, 0).xyz;
#endif

    float opacity;
#if defined(OPACITY_IS_UINTEGER)
    opacity = float(texelFetch(u_OPACITYSampler, texelCoord, 0).x);
#ifdef OPACITY_NEEDS_NORMALIZATION
    opacity = opacity / 65535.0; // normalize unsigned short
#endif
#else
    opacity = texelFetch(u_OPACITYSampler, texelCoord, 0).x;
#endif

    splat_center = (u_ModelMatrix * vec4(splat_center, 1.0)).xyz;

    vec4 view_splat_center = u_ViewMatrix * vec4(splat_center, 1.0);
    vec4 clip_splat_center = u_ProjectionMatrix * view_splat_center;
    clip_splat_center /= clip_splat_center.w; // perspective division

    mat3 C = computeC(scale, rotation);
    mat3 M = mat3(u_ModelMatrix);
    mat3 world_covariance = M * C * transpose(C) * transpose(M);
    vec3 camera_covariance = computeCameraCovariance(world_covariance, view_splat_center.xyz);

    float a = camera_covariance.x; // Variance x
    float b = camera_covariance.y; // Covariance xy
    float c = camera_covariance.z; // Variance y

	float det = (a * c - b * b);
	if (det == 0.0) {
		gl_Position = vec4(0.0);
        return;
    }
    
    float det_inv = 1.0 / det;
    // Calculate the inverse of the covariance matrix
	v_conic = vec3(c * det_inv, -b * det_inv, a * det_inv);
    
    // pow(e, pow(-3.4, 2) * -0.5) = 1/255, so 3.4 is the standard deviation in terms of the Gaussian falloff that results in a radius of 1 pixel when the variance is 1.
    // sqrt(a) and sqrt(c) are the standard deviations in x and y direction, so multiplying them with 3.4 gives us the radius in pixels where the Gaussian falloff results in 1/255 opacity.
    vec2 quad_pixel_size = vec2(3.4 * sqrt(a), 3.4 * sqrt(c));  // screen space half quad height and width
    vec2 quad_ndc_size = quad_pixel_size / vec2(u_FramebufferSize) * 2.0;  // in ndc space
    clip_splat_center.xy = clip_splat_center.xy + a_position * quad_ndc_size;
    v_uv = a_position * quad_pixel_size;
    gl_Position = clip_splat_center;

    v_color = vec4(calculateSphericalHarmonics(texelCoord), opacity);
}

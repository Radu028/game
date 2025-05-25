#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec3 fragNormal;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Custom lighting uniforms
uniform vec3 sunDirection;
uniform vec3 sunColor;
uniform float ambientStrength;
uniform vec3 viewPos;
uniform float time;

// Output fragment color
out vec4 finalColor;

void main()
{
    // Sample texture
    vec4 texelColor = texture(texture0, fragTexCoord);
    vec3 color = texelColor.rgb * colDiffuse.rgb;
    
    // Normalize inputs
    vec3 normal = normalize(fragNormal);
    vec3 lightDir = normalize(-sunDirection); // Sun direction points TO the sun
    
    // Enhanced ambient lighting with subtle time variation
    float ambientVariation = 1.0 + 0.1 * sin(time * 0.5);
    vec3 ambient = ambientStrength * sunColor * ambientVariation;
    
    // Diffuse lighting (Lambert) with enhanced contrast
    float diff = max(dot(normal, lightDir), 0.0);
    diff = pow(diff, 0.8); // Slightly soften the lighting transition
    vec3 diffuse = diff * sunColor;
    
    // Enhanced specular highlighting with Blinn-Phong
    vec3 viewDir = normalize(viewPos - fragPosition);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), 64.0);
    vec3 specular = spec * sunColor * 0.4; // Increased specular intensity
    
    // Improved shadow approximation with softer transitions
    float shadowFactor = 0.5 + 0.5 * max(dot(normal, lightDir), 0.0);
    shadowFactor = mix(0.7, 1.0, shadowFactor); // Softer shadow range
    
    // Add subtle rim lighting effect
    float rimDot = 1.0 - dot(viewDir, normal);
    float rimIntensity = pow(rimDot, 3.0) * 0.3;
    vec3 rimColor = sunColor * rimIntensity;
    
    // Combine all lighting components
    vec3 result = (ambient + diffuse + specular + rimColor) * color;
    result *= shadowFactor;
    
    finalColor = vec4(result, texelColor.a * colDiffuse.a);
}
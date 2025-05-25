#version 330

// Input vertex attributes
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;

// Input uniform values
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matView;
uniform mat4 matProjection;

// Output vertex attributes (to fragment shader)
out vec3 fragPosition;
out vec2 fragTexCoord;
out vec3 fragNormal;

void main()
{
    // Calculate world space position
    vec4 worldPos = matModel * vec4(vertexPosition, 1.0);
    
    // Send vertex attributes to fragment shader
    fragPosition = worldPos.xyz;
    fragTexCoord = vertexTexCoord;
    
    // Transform normal to world space
    mat3 normalMatrix = transpose(inverse(mat3(matModel)));
    fragNormal = normalize(normalMatrix * vertexNormal);
    
    // Calculate final vertex position
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
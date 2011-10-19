in vec3 vertex;
varying vec4 normal;

void main() {
  gl_Position = gl_ModelViewProjectionMatrix * vec4(vertex, 1.0);
  
  float Angle = radians(-90);
  mat4 RotationMatrix = mat4(
    cos( Angle ), sin( Angle ), 0.0, 0.0,
    -sin( Angle ),  cos( Angle ), 0.0, 0.0,
    0.0,           0.0,          1.0, 0.0,
    0.0,           0.0,          0.0, 1.0 );

  normal = RotationMatrix * vec4(gl_NormalMatrix * gl_Normal, 1.0);
}


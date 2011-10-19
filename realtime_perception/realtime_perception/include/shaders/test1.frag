varying vec4 normal;

void main(void)
{
  gl_FragData[0] = gl_Color;
  //gl_FragData[1] = vec4(1.0, 1.0, 0.0, 1.0);
  gl_FragData[1] = sin(dot (normal, vec4(0.0, 0.0, 0.0, 0.0)));
  gl_FragData[2] = vec4(
    (normal.y + 1.0) * 0.5,
    (normal.x + 1.0) * 0.5,
    (normal.z + 1.0) * 0.5,
    1.0);
}

		

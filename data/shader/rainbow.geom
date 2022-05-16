#version 460

uniform ivec4 info_values;

layout (points) in;
layout (points, max_vertices = 1) out;

in vec3 frag_world_position[1];
in vec4 frag_color[1];
out vec4 frag_color_geom;
flat out ivec4 frag_info;

void main()
{
    /*
    int x1 = int(frag_world_position[0].x);
    int y1 = int(frag_world_position[0].y);
    int z1 = int(frag_world_position[0].z);


    if ((x1 % 2 == 0 && y1 % 2 == 0 && z1 % 2 == 0) ||
        (x1 % 3 == 0 && y1 % 3 == 0 && z1 % 3 == 0) ||
        (x1 % 5 == 0 && y1 % 5 == 0 && z1 % 5 == 0) ||
        (x1 % 7 == 0 && y1 % 7 == 0 && z1 % 7 == 0))
    {
        EndPrimitive();
        return;
    }
    */

    gl_Position = gl_in[0].gl_Position;
    gl_PointSize = 0.1;
    frag_color_geom = vec4(0, 1, 0, 1);
    frag_info = info_values;
    
    EmitVertex();
    EndPrimitive();
}
package org.firstinspires.ftc.teamcode.ours;

public class milesCameraReadPseudo {
    public static int sample(Matrix<Vec3> camMatrix, int scale, int offset, int R, int G, int B, int precision) {
        Vec3 val;
        for (int y=0; y<=3; y+=1) {
            for (int x=0; x<=3; x+=1) {
                val += camMatrix.sample(x*scale+offset, y*scale+offset);
            }
        }

        int col = val.x/9;

        if((col>R-precision)&&(col<R+precision)) {
            return 0;
        }
        if((col>G-precision)&&(col<G+precision)) {
            return 1;
        }
        if((col>B-precision)&&(col<B+precision)) {
            return 2;
        }
    }
}

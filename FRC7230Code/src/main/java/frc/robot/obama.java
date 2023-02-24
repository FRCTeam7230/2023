public class obama {
    int i = 5;

    public void amongus(double x, double y){
        StdDraw.setPenRadius(0.01);
        StdDraw.setPenColor(StdDraw.RED);
        
        StdDraw.filledCircle(0.5 + x, 0.5 + y, 0.12);
        StdDraw.filledRectangle(0.5 + x, 0.4 + y, 0.12, 0.1);
        StdDraw.filledRectangle(0.416 + x, 0.25 + y, 0.036, 0.05);
        StdDraw.filledRectangle(0.584 + x, 0.25 + y, 0.036, 0.05);

        StdDraw.filledRectangle(0.65 + x, 0.44 + y, 0.05, 0.08);

        StdDraw.setPenColor(StdDraw.BLUE);
        StdDraw.filledEllipse(0.42 + x, 0.5 + y, 0.08, 0.05);

        
    }
}
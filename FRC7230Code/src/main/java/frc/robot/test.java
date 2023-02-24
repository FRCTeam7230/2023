import java.util.ArrayList;
import java.util.Scanner;


class test{
    public static void main(String args[]){  
        obama myObj = new obama();
        System.out.println(myObj.i);

        double posx = 0;
        double posy = 0;

        Scanner sc = new Scanner(System.in);
        
         

        while (true){
            for (int j = 0; j < 10; j++){
                StdDraw.clear();
                myObj.amongus(posx, posy);
    
                posy = posy + 0.02;
                StdDraw.pause(50);
            }

            for (int k = 0; k < 10; k++){
                StdDraw.clear();
                myObj.amongus(posx, posy);

                posy = posy - 0.02;
                StdDraw.pause(50);

            }
        
        



        }



    }
}
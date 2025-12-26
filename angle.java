//Vatamanu Matei - Calcul unghi in O(log(n)^2)

package org.firstinspires.ftc.teamcode;

/*
* Calculul unghiului theta necesar pentru a arunca mingea la cos
* */

public class angle {

    /*
    * G - constanta gravitationala
    * d - distanta de la robot la locul la care aruncati, calculata prin odometrie
    * u - viteza initiala a mingii cand este aruncata is metri/secunda (trebuie aproximata)
    * H - inaltimea cosului in metri
    * theta - unghiul la care este lansata bila
    * N - numarul de iteratii pe care le face cautarea binara; cu cat e mai mare cu atat unghiul va fi mai precis, dar va lua mai mult
    * */
    private final double G = 9.81;
    private final double H = 1;
    private final double u = 4.3;
    private final double N = 50;

    //Functia Height calculeaza inaltimea la care se va afla mingea la distanta d atunci cand
    // mingea este aruncata cu unghiul theta
    //https://openstax.org/books/university-physics-volume-1/pages/4-3-projectile-motion
    private double Height(double theta, double d) {
        return Math.tan(theta)*d - (G*d*d)/(2*u*u*Math.cos(theta)*Math.cos(theta));
    }

    //Derivata functiei Height in functie de theta
    //Aceasta ne va ajuta sa gasim unghiul la care inaltimea este maxima la distata d
    private double Height_theta_derivative(double d, double theta) {
        return d*(1/(Math.cos(theta)*Math.cos(theta))) - (d*d*G*Math.sin(theta)/(u*u*Math.cos(theta)*Math.cos(theta)*Math.cos(theta)));
    }

    //limit_search cauta binar unghiul pentru care derivata este 0
    //acolo stim ca functia height este maxima pentru distanta d.
    private double limit_search(double d) {
        double l = 0, r = Math.PI / 2;
        double m = (l + r) / 2;
        
        for(int i = 0; i < N; i++) {
            m = (l + r) / 2;
            if (Height_theta_derivative(m, d) > 0) {
                l = m;
            } else r = m;
        }
        return m;
    }

    //Intr-un final cautam binar unghiul theta pentru care la distanta d functia
    //Height va avea valoarea H.
    private double theta_search(double d) {
        double l = 0, r = limit_search(d);
        double m = (l + r) / 2;

        for(int i = 0; i < N; i++) {
            m = (l + r) / 2;
            if (Height(m, d) > H) {
                l = m;
            } else r = m;
        }
        return m;
    }

    //functie care transforma unghiul theta in valoarea care trebuie tsrimisa catre servo.
    //Aici trebuie schimbat in functie de configuratia voastra;
    private double radians_to_ticks(double theta) {
        return 2*theta/Math.PI;
    }

    public double get_theta(double d) {
        double theta = theta_search(d);
        return radians_to_ticks(theta);
    }

}

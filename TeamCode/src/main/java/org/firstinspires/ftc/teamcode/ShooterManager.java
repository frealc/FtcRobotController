package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterManager {

    private DcMotorEx lanceur1, lanceur2;
    private Servo pousseballe;
    private CRServo roue_a_balle, attrapeBalle;

    // --- Paramètres de tir ---
    private double targetTicks = 1615; // vitesse voulu
    private double maxTicks = 2000;    // vitesse max
    private long cadenceTir = 250;     // ms entre deux tirs
    private long tempsStableNecessaire = 80; // ms stabilité avant tir

    private long dernierTir = 0;
    private long debutStabilite = 0;
    public boolean pretATirer = false;

    public ShooterManager(DcMotorEx l1, DcMotorEx l2, Servo pousse,
                          CRServo attrape, CRServo roue) {
        lanceur1 = l1;
        lanceur2 = l2;
        pousseballe = pousse;
        attrapeBalle = attrape;
        roue_a_balle = roue;


        lanceur1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lanceur2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



    }


    public void update() {


        double v1 = lanceur1.getVelocity();
        double v2 = lanceur2.getVelocity();
        double vitesseActuelle = (v1 + v2) / 2.0;


        if (Math.abs(vitesseActuelle - targetTicks) <= 20) { // ±20 ticks
            if (debutStabilite == 0)
                debutStabilite = System.currentTimeMillis();

            pretATirer = System.currentTimeMillis() - debutStabilite >= tempsStableNecessaire;

        } else {
            debutStabilite = 0;
            pretATirer = false;
        }

        if (!pretATirer && System.currentTimeMillis() - dernierTir >= cadenceTir) {
            pretATirer = true;
        }

        // Tir auto
        if (pretATirer && System.currentTimeMillis() - dernierTir >= cadenceTir) {




            dernierTir = System.currentTimeMillis();
        }
    }


    public void startShooter(double targetTicks) {
        this.targetTicks = targetTicks;

        // Conversion en puissance
        double power = targetTicks / maxTicks;
        lanceur1.setPower(power);
        lanceur2.setPower(power);

        debutStabilite = 0;
        pretATirer = false;
        dernierTir = System.currentTimeMillis();
    }


    public void stopShooter() {
        lanceur1.setPower(0);
        lanceur2.setPower(0);
        pretATirer = false;
        debutStabilite = 0;
    }


    public double getVitesseActuelle() {
        return (lanceur1.getVelocity() + lanceur2.getVelocity()) / 2.0;
    }
}

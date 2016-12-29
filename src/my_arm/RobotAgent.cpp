#include "RobotAgent.h"
#include <math.h>

#include "GeopadQMLAdapter.h"
#include "KsGlobal.h"
#include "K3DStateMachine.h"

K3DSMRule<RobotAgent> RobotAgent::_stateMachine[RobotAgent::RB_STATE_MACHINE_TOTAL] = {
    /* 0 */ { &RobotAgent::fTrue,          false, &RobotAgent::init,    0, 1 },
    /* 1 */ { &RobotAgent::checkIdle,      false, &RobotAgent::operate, 0, 2 },
    /* 2 */ { &RobotAgent::checkOperating, false, &RobotAgent::goIdle,  1, 1 }
};

void RobotAgent::startOperation(RobotAgent* robotAgent)
{
    if (robotAgent != nullptr) {
        int currentStateId = K3DRobotMachine<RobotAgent>::run(robotAgent);
        if (currentStateId > 0) {
            robotAgent->setCurrentStateRuleId(currentStateId);
        }
    }
}

bool RobotAgent::checkIdle()
{
    QVariant var;
    K3D_THREAD_BLOCKING_INVOKE_RET(_itemUI, getState, var)
    return var.toInt() == 1;
}

bool RobotAgent::checkOperating()
{
    QVariant var;
    K3D_THREAD_BLOCKING_INVOKE_RET(_itemUI, getState, var)
    return var.toInt() == 2;
}

void RobotAgent::operate() 
{
    GEOPAD_QML_ITEM_LOCAL_INVOKE_I(setState, 2);
}

void RobotAgent::goIdle()
{
    GEOPAD_QML_ITEM_LOCAL_INVOKE_I(setState, 1);
}

//donne la distance du robot % aux point p
float RobotAgent::getDistance(QPoint deb,QPoint end){
    return (float) sqrt ((deb.x() - end.x()) * (deb.x() - end.x()) + (deb.y() - end.y()) * (deb.y() - end.y()) );
}

//faire une rotation degree du robot
void RobotAgent::rotate(int degree){
    CurentRotation+=degree ;
    CurentRotation=CurentRotation % 360;
//	printf("new rotat\t");getInfo();
}

//deplase le robot +distance
void RobotAgent::move(int distance){
    CurentPosition.setX(CurentPosition.x() + (int) (cos(this->CurentRotation*3.141593/180) * distance));
    CurentPosition.setY(CurentPosition.y() + (int) (sin(this->CurentRotation*3.141593/180) * distance));

//	printf("new Move\t");getInfo();

}

int RobotAgent::getCapteurDistance(int numCapter,int Sens){
    return 0;
#if 0

    //droite portant le chanp de vision du capteur
    // y =   tang(ongle) x + b
    // b = y-tang(ongle) x
    // d'out
    // y =   tan(CurentRotation) x + CurentPosition.y - tang(ongle) CurentPosition.x
    //

    //cas d'un obstacle
    // intersection avec un obstacle
    // obstacle(xmim,ymin) (xmax,ymax)

    // 4 point d'intersection
    // on choisit qui satisfier les condition suivant
    //
    //  1: xmin < x < xmax
    //  2: ymin < y < ymax
    //  3: le plus pres du robot

    // xmin, tan(CurentRotation) xmin + CurentPosition.y - tang(ongle) CurentPosition.x
    // xmax, tan(CurentRotation) xmax + CurentPosition.y - tang(ongle) CurentPosition.x
    // (ymin - CurentPosition.y + tang(ongle) CurentPosition.x )/ tan(CurentRotation) ,ymin
    // (ymax - CurentPosition.y + tang(ongle) CurentPosition.x )/ tan(CurentRotation) ,ymax

    //generalisation
    //il faut prendre enconsederation que le lobstacle le plus pres

//Debut Algorithme
    //pour un capteur --> distancePositif ,distanceNegatif

    //calcule du vecteur pour trouver le sens



    double CateurCurentRotation = CurentRotation + (360/(nbrCapteur)) * numCapter ;

     float senspositif_x= (float) CurentPosition.x() + (float) (10*cos(CateurCurentRotation*3.141593/180) );
     float senspositif_y= (float) CurentPosition.y() + (float) (10*sin(CateurCurentRotation*3.141593/180) );


    float distPositif = 99999.;int solPositif  = -1;
    float distNigatif = 99999.;int solNigatif  = -1;
    float delta =0.1f;

    float sens;

    //pour chaque obstacle
    for (int i = 0; i < envrnemt.size(); i++) {
        obstacle *obst = envrnemt[i];
        if(obst == nullptr)
            continue;
    // for (int k=0;k<nbreObstacle;k++){
        QPoint *p[4];
        p[0]=new QPoint( obst->min.x() , tan(CateurCurentRotation*3.141593/180) * obst->min.x() + CurentPosition.y() - tan(CateurCurentRotation*3.141593/180) * CurentPosition.x());
        p[1]=new QPoint( obst->max.x() , tan(CateurCurentRotation*3.141593/180) * obst->max.x() + CurentPosition.y() - tan(CateurCurentRotation*3.141593/180) * CurentPosition.x());
        p[2]=new QPoint((obst->min.y() - CurentPosition.y() + tan(CateurCurentRotation*3.141593/180) * CurentPosition.x() )/ tan(CateurCurentRotation*3.141593/180) ,obst->min.y());
        p[3]=new QPoint((obst->max.y() - CurentPosition.y() + tan(CateurCurentRotation*3.141593/180) * CurentPosition.x() )/ tan(CateurCurentRotation*3.141593/180) ,obst->max.y());

        float dist = 99999.;
        int sol  = -1;

        //pour chaque droite du obstacle
        for (int i=0;i<4;i++){

        //	printf("point sol (%f,%f) dist =%f\n",p[i]->x,p[i]->y,p[i]->getDistance(Point (CurentPosition.x,CurentPosition.y)));
            if ((((obst->min.y()-delta <=p[i]->y())) && (p[i]->y() <= (obst->max.y() +delta))) &&
                (((obst->min.x()-delta) <=p[i]->x()) && (p[i]->x() <= (obst->max.x() +delta))) )
            {
                //choisir la plus proche des intersection
                if ((dist >= getDistance(*p[i],QPoint ( CurentPosition.x(), CurentPosition.y())))
                    && (getDistance(*p[i],QPoint (CurentPosition.x(),CurentPosition.y())) >=0.001))
                {
                    sol  = i;
                    dist = getDistance(*p[i],QPoint (CurentPosition.x(),CurentPosition.y()));
                }
            }
        }

        //si existe une solution
        if (sol>=0){

          sens = (senspositif_x - CurentPosition.x()) *(p[sol]->x() - CurentPosition.x())
                +(senspositif_y - CurentPosition.y()) *(p[sol]->y() - CurentPosition.y()) ;
         if (sens==0)
             sol=0;
          if (sens>0) {
              if (dist < distPositif){
                //solPositif =k;
                distPositif =dist;
              }
        //	  printf(" sens +");

          }
          else {
              if (dist < distNigatif){
                //solNigatif =k;
                distNigatif =dist;
              }
        //	  printf(" sens -");

          }
          //printf(" obstacle %d la solution [%d] distance=%f \n",k+1,sol,dist);
        }
    //	else
          //printf(" obstacle %d pas dintersection \n",k+1);
    }


        // printf(" obstacle %d  distance=+ %f \n",solPositif+1,distPositif);
        // printf(" obstacle %d  distance=- %f \n",solNigatif+1,distNigatif);

    if (Sens) return distPositif;
    else  return distNigatif;
#endif
}

void RobotAgent::getInfo(){

//	printf("Robot pos init = (%f,%f) rot init = %d \n",CurentPosition.x,CurentPosition.y, CurentRotation);
}

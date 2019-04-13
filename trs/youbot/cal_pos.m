function [theta,object_x,object_y]=cal_pos()
    O=[-3,-6];
    obj=[-3,-3.2,-3.35,-3.275,-2.75;
        -5.65,-5.925,-6,-6.15,-5.8];
    car_pos=[-3.167,-5.25];
    theta=[0,0,0,0,0];
    object_x=[0,0,0,0,0];
    object_y=[0,0,0,0,0];
    R=[0,0,0,0,0];
    for i=1:5
        if obj(2,i)>-6
            theta(i)=-pi/2+atan((-3-obj(1,i))/(6+obj(2,i)));
        else
            theta(i)=atan((-6-obj(2,i))/(-3-obj(1,i)));
        end
    end
    R2=sqrt((-3.167+3)^2+(6-5.25)^2);
    for i=1:5
        R(i)=sqrt((-3-obj(1,i))^2+(-6-obj(2,i))^2)*R2/0.35;
    end
    alpha=atan(0.167/0.75);
    for i=1:5
        if obj(2,i)<-6
            object_x(i)=-3-R(i)*cos(theta(i)+alpha);
            object_y(i)=-6+R(i)*sin(theta(i)+alpha);
        else
            object_x(i)=-3-R(i)*cos(-theta(i)-alpha);
            object_y(i)=-6+R(i)*sin(-theta(i)-alpha);
        end
    end
    object_x
    object_y
    a=sqrt((-3-object_x(:)).^2+(-6-object_y(:)).^2)
end
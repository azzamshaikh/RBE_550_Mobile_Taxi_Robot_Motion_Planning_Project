#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like ModelPlugin, event
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>        // to access Vector3d() from ignition math class
#include <ignition/math.hh>
#include <stdio.h>

namespace gazebo {
    class MovementPlugin : public ModelPlugin {
        
        public:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
                
                // Store the pointer to the model
                this->model = _model;

                std::cout<< "Model Name=" << this->model->GetName() << std::endl;

                // Assign a default value
                this->start_x = 1;         
                this->start_y = 0;
                this->start_z = 0;
                this->end_x = 1;
                this->end_y = 1;
                this->end_z = 0;
                this->time = 30;

                if (_sdf->HasElement("start_x")) // check if element existence 
                {
                    this->start_x = _sdf->Get<double>("start_x");  // use _sdf pointer & Get to find value in <model_vel>
                   
                }
                
                if (_sdf->HasElement("start_y")) // check if element existence 
                {
                    this->start_y = _sdf->Get<double>("start_y");  // use _sdf pointer & Get to find value in <model_vel>
                   
                }

                if (_sdf->HasElement("start_z")) // check if element existence 
                {
                    this->start_z = _sdf->Get<double>("start_z");  // use _sdf pointer & Get to find value in <model_vel>
                   
                }

                if (_sdf->HasElement("end_x")) // check if element existence 
                {
                    this->end_x = _sdf->Get<double>("end_x");  // use _sdf pointer & Get to find value in <model_vel>
                   
                }

                if (_sdf->HasElement("end_y")) // check if element existence 
                {
                    this->end_y = _sdf->Get<double>("end_y");  // use _sdf pointer & Get to find value in <model_vel>
                   
                }

                if (_sdf->HasElement("end_z")) // check if element existence 
                {
                    this->end_z = _sdf->Get<double>("end_z");  // use _sdf pointer & Get to find value in <model_vel>
                   
                }

                if (_sdf->HasElement("time")) // check if element existence 
                {
                    this->time = _sdf->Get<double>("time");  // use _sdf pointer & Get to find value in <model_vel>
                   
                }

                std::cout << "start = " << this->start_x << " " << this->start_y << " " << this->start_z << std::endl;
                std::cout << "end = " << this->end_x << " " << this->end_y << " " << this->end_z << std::endl;
                std::cout << "time =" << this->time << std::endl;


                gazebo::common::PoseAnimationPtr anim(
                    
                    new gazebo::common::PoseAnimation("robot_movement",60.0,true));

                gazebo::common::PoseKeyFrame *key;

                // starting location
                key = anim->CreateKeyFrame(0);
                key->Translation(ignition::math::Vector3d(this->start_x,this->start_y,this->start_z));
                key->Rotation(ignition::math::Quaterniond(0,0,0));


                // set waypoint location after x seconds
                key = anim->CreateKeyFrame(this->time);
                key->Translation(ignition::math::Vector3d(this->end_x, this->end_y, this->end_z));
                key->Rotation(ignition::math::Quaterniond(0,0,0));

                // return to start after 2x seconds
                key = anim->CreateKeyFrame(this->time * 2);
                key->Translation(ignition::math::Vector3d(this->start_x,this->start_y,this->start_z));
                key->Rotation(ignition::math::Quaterniond(0,0,0));

                _model->SetAnimation(anim);
            }

        // Data members
        private:
            physics::ModelPtr model;  // Pointer to the model

        private:
            double start_x;               
            double start_y;
            double start_z;
            double end_x;
            double end_y;
            double end_z;
            double time;
    };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MovementPlugin)
} 
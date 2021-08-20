#include <ros/ros.h>
#include <kinematics_action_msg/inverse_kinematics_msgAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <angles/angles.h>

class InverseKinematicsAction {
    
    protected:

        ros::NodeHandle nh_;

        /**
        * Definizione action server.
        */    
        actionlib::SimpleActionServer<kinematics_action_msg::inverse_kinematics_msgAction> action_server_;

        /**
        * Definizione RobotModelLoader, RobotModel e JointModel
        */
        std::vector<std::vector<double>> ik_solutions_;
        robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
        robot_model::RobotModelConstPtr kinematic_model_;
        const robot_state::JointModelGroup * jmg_;
            
    public:

        /**
        * Costruttore
        */
        InverseKinematicsAction():
            /**
            * Costruttore per un action server. Riceve in ingresso la node handle, il nome del server, la callback
            * per il raggiungimento dell'obiettivo, un booleano per l'autostart.
            */
            action_server_(nh_, "inverse_kinematics_server", boost::bind(&InverseKinematicsAction::goal_achived_, this, _1), false),
            
            /**
            * Costruzione del RobotModel e del JointModelGroup.
            */
            robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description")),
            kinematic_model_(robot_model_loader_->getModel()),
            jmg_(nullptr) 
            {
                // Get the planning group name from the parameter server
                std::string planning_group_name;

                if(!nh_.getParam("planning_group_name", planning_group_name)) {
                    ROS_ERROR("'planning_group_name' is undefined on the parameter server");
                    return;
                }

                jmg_ = kinematic_model_->getJointModelGroup(planning_group_name);

                /**
                * Avvio del server.
                */
                action_server_.start();
            }
        /**
        * Distruttore.
        */
        ~InverseKinematicsAction(){}

        void goal_achived_(const kinematics_action_msg::inverse_kinematics_msgGoalConstPtr & goal) {
            
            ROS_INFO("IK server: goal ricevuto");

            /**
            * Calcolo del Solver cinematico dal JointModelGroup.
            */
            const kinematics::KinematicsBaseConstPtr solver = jmg_->getSolverInstance();

            kinematics_action_msg::inverse_kinematics_msgResult ik_result;

            int ik_calls_counter = 0;

            while(ik_calls_counter < 2000 && ros::ok()) {

                std::vector<double> seed_state = generateSeedState_();
                std::vector<double> solution;
                moveit_msgs::MoveItErrorCodes result;

                /**
                * Data una posa dell'end-effector, calcola le soluzioni degli angoli dei giunti che 
                * consentono il raggiungimento della posa.
                *
                * Il seed_state rappresenta una soluzione randomica iniziale per la cinematica inversa.
                * La solution è un vettore di vettori in cui ogni entry rappresenta una soluzione valida dei giunti.
                * result rappresenta il risultato della query.
                */
                solver->getPositionIK(goal->end_effector_pose, seed_state, solution, result);

                if(result.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

                    /**
                    * Normalizzazione delle soluzioni.
                    */
                    normalizeJointPositions_(solution);

                    if(isSolutionNew_(solution)) {
                        /**
                        * Inserimento della soluzione nel vettore delle soluzioni.
                        */
                        ik_solutions_.push_back(solution);

                        /**
                        * Aggiornamento del RobotState sulla base della soluzione corrente.
                        */
                        moveit::core::RobotState robot_state(kinematic_model_);
                        robot_state.setVariablePositions(solution);

                        /**
                        * Invio della soluzione corrente come feedback.
                        */
                        kinematics_action_msg::inverse_kinematics_msgFeedback feedback;
                        moveit::core::robotStateToRobotStateMsg(robot_state, feedback.ik_feedback);
                        action_server_.publishFeedback(feedback);

                        ik_result.ik_solutions.push_back(feedback.ik_feedback);
                    }
                }

                ik_calls_counter++;
            }

            if(ik_solutions_.size() == 0)
                action_server_.setAborted(ik_result, "Non è possibile trovare alcuna soluzione");
            else {
                std::ostringstream ss;
                ss << "Trovate " << ik_solutions_.size() << " IK soluzioni";
                
                action_server_.setSucceeded(ik_result, ss.str());
            }

            ik_solutions_.resize(0);
        }

        /**
        * Genera una soluzione di cinematica inversa iniziale casuale.
        */
        std::vector<double> generateSeedState_() const {
            std::vector<double> seed_state;
            
            std::vector<std::string> joint_names = kinematic_model_->getVariableNames();

            for(int i=0; i < joint_names.size(); i++) {
                double lb = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->lower;
                double ub = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->upper;
                double span = ub-lb;
                
                seed_state.push_back((double)std::rand()/RAND_MAX * span + lb);
            }

            return seed_state;
        }

        /**
        * Se la soluzione è di un giunto rotoidale viene effettuata la normalizzazione degli angoli.
        */
        void normalizeJointPositions_(std::vector<double> & solution) const {
            
            for(int i = 0; i < solution.size(); i++) {
                if (jmg_->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE) {
                    solution[i] = angles::normalize_angle(solution[i]);
                }
            }
        }

        bool isSolutionNew_(const std::vector<double> & solution) const {
    
            for(int i=0; i < ik_solutions_.size(); i++) {
                bool are_solutions_equal = true;

                for(int j=0; j < ik_solutions_[i].size() && are_solutions_equal; j++) {
                    double diff;

                    if(jmg_->getActiveJointModels()[j]->getType() == robot_model::JointModel::REVOLUTE) {
                        diff = angles::shortest_angular_distance(ik_solutions_[i][j], solution[j]);
                    }
                    else {
                        diff = ik_solutions_[i][j] - solution[j];
                    }

                    if(std::fabs(diff) > 1e-3)
                        are_solutions_equal = false;
                }

                if(are_solutions_equal)
                    return false;
            }

            return true;
        }
    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinematics_action_server");
    
    /**
    * Creazione del server.
    */
    InverseKinematicsAction ik_action;

    ROS_INFO("Server di inversione cinematica avviato");

    ros::spin();

    ros::shutdown();
    return 0;
}
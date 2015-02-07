/*
 * A Python wrapper to SCISim library.
 */

#include "PySCISim.h"

#include "SCISim/Utilities.h"
#include "ThreeDRigidBodies/Constraints/BoxBoxUtilities.h"
#include "ThreeDRigidBodies/Constraints/StapleStapleUtilities.h"
#include "ThreeDRigidBodies/Constraints/MeshMeshUtilities.h"
#include "ThreeDRigidBodies/Constraints/SphereSphereConstraint.h"
#include "ThreeDRigidBodies/Constraints/TeleportedSphereSphereConstraint.h"
#include "ThreeDRigidBodies/Constraints/SphereBodyConstraint.h"
#include "ThreeDRigidBodies/Constraints/BodyBodyConstraint.h"
#include "ThreeDRigidBodies/Constraints/StaticPlaneBodyConstraint.h"
#include "ThreeDRigidBodies/Constraints/StaticPlaneBoxConstraint.h"
#include "ThreeDRigidBodies/Constraints/StaticPlaneSphereConstraint.h"
#include "ThreeDRigidBodies/Constraints/StaticCylinderSphereConstraint.h"
#include "ThreeDRigidBodies/Constraints/KinematicObjectSphereConstraint.h"
#include "ThreeDRigidBodies/Constraints/CollisionUtilities.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Geometry>
#include <cmath>

using namespace std;


////////////////////////////////////////////
// SCISim
////////////////////////////////////////////
SCISim::SCISim(): MAGIC_BINARY_NUMBER(9035768) {
    m_iteration = 0;
    m_unconstrained_map = std::unique_ptr<UnconstrainedMap>({ nullptr });
    m_end_time = SCALAR_NAN;
    m_impact_operator = std::unique_ptr<ImpactOperator>({ nullptr });
    m_CoR = SCALAR_NAN;
    m_friction_operator = std::unique_ptr<FrictionOperator>({ nullptr });
    m_mu = SCALAR_NAN;
    m_impact_friction_map = std::unique_ptr<ImpactFrictionMap>({ nullptr });
    m_scripting_callback = std::unique_ptr<RigidBodyScriptingCallback>({ nullptr });
    
    m_output_forces = false;
    // Number of timesteps between saves
    m_steps_per_save = -1;
    // Number of saves that been conducted so far
    m_output_frame = 0;
    m_dt_string_precision = -1;
    m_save_number_width = -1;
    
    m_serialize_snapshots = false;
    m_overwrite_snapshots = true;
}

void SCISim::run(const std::string& xml_scene_file_name) {
    // load a scene if given
    if (xml_scene_file_name != "") {
        openScene(xml_scene_file_name);
    }
}

unsigned SCISim::computeTimestepDisplayPrecision( const Rational<std::intmax_t>& dt, const std::string& dt_string )
{
    if( dt_string.find( '.' ) != std::string::npos )
    {
        return unsigned( StringUtilities::computeNumCharactersToRight( dt_string, '.' ) );
    }
    else
    {
        std::string converted_dt_string;
        std::stringstream ss;
        ss << std::fixed << scalar( dt );
        ss >> converted_dt_string;
        return unsigned( StringUtilities::computeNumCharactersToRight( converted_dt_string, '.' ) );
    }
}

bool SCISim::openScene(const std::string& xml_scene_file_name, unsigned fps, bool render_at_fps, bool lock_camera) {
    // Simulation data to load
    RigidBodySimState new_sim_state;
    std::string new_scripting_callback_name;
    
    // Attempt to load the scene
    {
        std::string new_dt_string;
        RenderingState UNUSED_rendering_state_UNUSED;
        
        const bool loaded_successfully = XMLSceneParser::parseXMLSceneFile( xml_scene_file_name, new_scripting_callback_name, new_sim_state, m_unconstrained_map, new_dt_string, m_dt, m_end_time, m_impact_operator, m_CoR, m_friction_operator, m_mu, m_impact_friction_map, UNUSED_rendering_state_UNUSED );
        if( !loaded_successfully )
        {
            return false;
        }
        
        m_dt_string_precision = computeTimestepDisplayPrecision( m_dt, new_dt_string );
    }
    
    m_sim.setState( new_sim_state );
    m_sim.clearConstraintCache(); // <- TODO: probs not needed, but won't hurt
    
    m_scripting_callback = KinematicScripting::initializeScriptingCallback( new_scripting_callback_name );
    assert( m_scripting_callback != nullptr );

    // store initial state
    m_sim_state_backup = m_sim.getState();
    
    // read data from sim
    updateSimData();
    
    return true;

}

int SCISim::stepSystem() {
    START_TIMER( "step_system" )
    
    const unsigned next_iter = m_iteration + 1;
    
//    HDF5File force_file;
//    assert( m_steps_per_save != 0 );
//    if( m_output_forces && m_iteration % m_steps_per_save == 0 )
//    {
//        assert( !m_output_dir_name.empty() );
//        const std::string constraint_force_file_name = generateOutputConstraintForceDataFileName();
//        std::cout << "Saving forces at time " << generateSimulationTimeString() << " to " << constraint_force_file_name << std::endl;
//        try
//        {
//            force_file.open( constraint_force_file_name, HDF5File::READ_WRITE );
//            // Save the simulation time
//            force_file.writeScalar( "", "t", m_iteration * scalar( m_dt ) );
//            // Save out the git hash
//            force_file.writeString( "", "git_hash", CompileDefinitions::GitSHA1 );
//            // Save the real time
//            //force_file.createGroup( "/run_stats" );
//            //force_file.writeString( "/run_stats", "real_time", TimeUtils::currentTime() );
//        }
//        catch( const std::string& error )
//        {
//            std::cerr << error << std::endl;
//            return EXIT_FAILURE;
//        }
//    }
    
    assert( m_scripting_callback != nullptr );
    m_scripting_callback->setRigidBodySimState( m_sim.state() );
    m_scripting_callback->startOfStepCallback( next_iter, m_dt );
    
    if( m_unconstrained_map == nullptr && m_impact_operator == nullptr && m_friction_operator == nullptr && m_impact_friction_map == nullptr )
    {
        // Nothing to do
    }
    else if( m_unconstrained_map != nullptr && m_impact_operator == nullptr && m_friction_operator == nullptr && m_impact_friction_map == nullptr )
    {
        m_sim.flow( next_iter, scalar( m_dt ), *m_unconstrained_map );
    }
    else if( m_unconstrained_map != nullptr && m_impact_operator != nullptr && m_friction_operator == nullptr && m_impact_friction_map == nullptr )
    {
        assert( m_scripting_callback != nullptr );
        m_sim.flow( *m_scripting_callback, next_iter, scalar( m_dt ), *m_unconstrained_map, *m_impact_operator, m_CoR );
    }
    else if( m_unconstrained_map != nullptr && m_impact_operator != nullptr && m_friction_operator != nullptr && m_impact_friction_map != nullptr )
    {
        assert( m_scripting_callback != nullptr );
        m_sim.flow( *m_scripting_callback, next_iter, scalar( m_dt ), *m_unconstrained_map, *m_impact_operator, m_CoR, *m_friction_operator, m_mu, *m_impact_friction_map );
    }
    else
    {
        std::cerr << "Impossible code path hit in stepSystem. This is a bug. Exiting." << std::endl;
        return EXIT_FAILURE;
    }
    
    ++m_iteration;
    
    STOP_TIMER( "step_system" )
    
    // Output timers, if enabled
#ifdef TIMERS_ENABLED
    {
        const std::string output_timer_file_name = m_output_dir_name.empty() ? "timing.txt" : m_output_dir_name + "/timing.txt";
        std::ofstream timer_stream( output_timer_file_name, std::ios::app );
        if( !timer_stream.is_open() )
        {
            std::cerr << "Error, failed to open timer log file: " << output_timer_file_name << std::endl;
            return EXIT_FAILURE;
        }
        SAVE_TIMERS( timer_stream )
    }
#endif
    
    CLEAR_TIMERS()
    
    // read updated system data
    updateSimData();
    
    return EXIT_SUCCESS;
}

void SCISim::resetSystem() {
    m_sim.setState( m_sim_state_backup );
    //m_sim.resetCachedData();
    m_sim.clearConstraintCache();
    if( m_impact_friction_map != nullptr ) m_impact_friction_map->resetCachedData();
}

void SCISim::updateSimData() {
    time = m_iteration * scalar( m_dt );
    
    T = m_sim.computeKineticEnergy();
    U = m_sim.computePotentialEnergy();
    
    p = m_sim.computeTotalMomentum();
    L = m_sim.computeTotalAngularMomentum();

    m_sim.computeNumberOfCollisions( number_of_collisions, collision_penetration_depth );
}

double SCISim::getSim_time() {
    return time;
}

double SCISim::getSim_T() {
    return T;
}

double SCISim::getSim_U() {
    return U;
}

Eigen::Vector3d SCISim::getSim_p() {
    return p;
}

Eigen::Vector3d SCISim::getSim_L() {
    return L;
}

unsigned int SCISim::getSim_collisions() {
    return number_of_collisions;
}

double SCISim::getSim_collisions_penetration_depth() {
    return collision_penetration_depth;
}

ThreeDRigidBodySim* SCISim::getSim_sim() {
    return &m_sim;
}

RigidBodySimState* SCISim::getSim_sim_state_backup() {
    return &m_sim_state_backup;
}

MatrixXs SCISim::getSimState_M() {
    return m_sim.getState().M();
}

VectorXs SCISim::getSimState_q() {
    return m_sim.getState().q();
}

VectorXs SCISim::getSimState_v() {
    return m_sim.getState().v();
}

VectorXs SCISim::getSimState_T() {
    const RigidBodySimState& m_sim_state = m_sim.getState();
    int N = m_sim_state.nbodies();
    VectorXs T(N);
    
    VectorXs T_helper = 0.5
    * m_sim_state.v().cwiseProduct(m_sim_state.M() * m_sim_state.v());
    
    // sum angular and linear kinetic energy
    for (int body = 0; body < N; ++body) {
        T(body) = T_helper.segment<3>(3 * body).sum()
        + T_helper.segment<3>(3 * N + 3 * body).sum();
    }
    
    return T;
}

VectorXs SCISim::getSimState_U() {
    const RigidBodySimState& m_sim_state = m_sim.getState();
    int N = m_sim_state.nbodies();
    VectorXs U = VectorXs::Zero(N);
    
    for (int body = 0; body < N; ++body) {
        VectorXs cur_q(12);
        cur_q.segment<3>(0) = m_sim_state.q().segment<3>(3*body);
        cur_q.segment<9>(3) = m_sim_state.q().segment<9>(3*N+9*body);
        MatrixXs M = this->getSimState_M();
        SparseMatrixsc cur_M(6, 6);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                cur_M.insert(i, j) = M(3*body+i, 3*body+j);
                cur_M.insert(i+3, j+3) = M(3*N+3*body+i, 3*N+3*body+j);
            }
        for (std::vector<Force*>::size_type i = 0; i < m_sim_state.forces().size();
             ++i) {
            U(body) += m_sim_state.forces()[i]->computePotential(
                                                                 cur_q, cur_M);
        }
    }
    
    return U;
}

VectorXs SCISim::getSimState_p() {
    const RigidBodySimState& m_sim_state = m_sim.getState();
    int N = m_sim_state.nbodies();
    VectorXs p(N * 3);
    
    for (int body = 0; body < N; ++body) {
        p.segment<3>(3 * body) = m_sim_state.getTotalMass(body)
        * m_sim_state.v().segment<3>(3 * body);
    }
    
    return p;
}

VectorXs SCISim::getSimState_L() {
    const RigidBodySimState& m_sim_state = m_sim.getState();
    int N = m_sim_state.nbodies();
    VectorXs L = VectorXs::Zero(N * 3);
    
    // Contribution from center of mass
    for (int body = 0; body < m_sim_state.nbodies(); ++body) {
        L.segment<3>(3 * body) += m_sim_state.getTotalMass(body)
        * m_sim_state.q().segment<3>(3 * body).cross(
                                                     m_sim_state.v().segment<3>(3 * body));
    }
    
    // Contribution from rotation about center of mass
    for (int body = 0; body < m_sim_state.nbodies(); ++body) {
        L.segment<3>(3 * body) += m_sim_state.getInertia(body)
        * m_sim_state.v().segment<3>(
                                     3 * m_sim_state.nbodies() + 3 * body);
    }
    
    return L;
}

int SCISim::getSimState_nbodies() {
    const RigidBodySimState& m_sim_state = m_sim.getState();
    return m_sim_state.nbodies();
}

void SCISim::setSimState_q(const VectorXs& q) {
    const RigidBodySimState& m_sim_state = m_sim.getState();
    VectorXs& cur_q = const_cast< VectorXs& >(m_sim_state.q());
    
    if ((q.cols() != cur_q.cols()) || (q.rows() != cur_q.rows()))
        throw "Wrong dimensions of q";
    
    cur_q = q;
}

void SCISim::setSimState_v(const VectorXs& v) {
    const RigidBodySimState& m_sim_state = m_sim.getState();
    VectorXs& cur_v = const_cast< VectorXs& >(m_sim_state.v());
    
    if ((v.cols() != cur_v.cols()) || (v.rows() != cur_v.rows()))
        throw "Wrong dimensions of v";
    
    cur_v = v;
}

VectorXs SCISim::get_x_from_q(const VectorXs& q) {
    int N = this->getSimState_nbodies();
    VectorXs x = VectorXs::Zero(N*6);
    
    // copy translation as is
    x.block(0, 0, 3*N, 1) = q.block(0, 0, 3*N, 1);
    // convert rotation matrix into axis/angle
    for (int n = 0; n < N; n++) {
        const Matrix33sr R = Eigen::Map<const Matrix33sr>( q.segment<9>( 3 * N + 9 * n ).data() );
        Matrix33sr log_R = R.log();
        x(3 * N + 3 * n + 0, 0) = log_R(2, 1);
        x(3 * N + 3 * n + 1, 0) = log_R(0, 2);
        x(3 * N + 3 * n + 2, 0) = log_R(1, 0);
    }
    
    return x;
}

// converts
VectorXs SCISim::get_q_from_x(const VectorXs& x) {
    int N = this->getSimState_nbodies();
    // check for correct input size
    if (x.rows() != N*6)
        throw "Wrong dimensions of x";
    
    VectorXs q = VectorXs::Zero(N*12);
    
    // copy translation as is
    q.block(0, 0, 3*N, 1) = x.block(0, 0, 3*N, 1);
    // convert rotation matrix into axis/angle
    for (int n = 0; n < N; n++) {
        Matrix33sr log_R = Matrix33sr::Zero();
        
        log_R(0, 1) = -x(3 * N + 3 * n + 2, 0);
        log_R(0, 2) = x(3 * N + 3 * n + 1, 0);
        log_R(1, 0) = x(3 * N + 3 * n + 2, 0);
        log_R(1, 2) = -x(3 * N + 3 * n + 0, 0);
        log_R(2, 0) = -x(3 * N + 3 * n + 1, 0);
        log_R(2, 1) = x(3 * N + 3 * n + 0, 0);
        
        Matrix33sr R = log_R.exp();
        q.segment<9>( 3 * N + 9 * n ) = Eigen::Map< VectorXs >(R.data(), R.size());
    }
    
    return q;
}

VectorXs SCISim::interpolate_x(const VectorXs& x0, const VectorXs& x1, scalar t) {
    if (t <= 0.0)
        return x0;
    if (t >= 1.0)
        return x1;
    
    int N = this->getSimState_nbodies();
    VectorXs x = VectorXs::Zero(N*6);
    
    // interpolate translation
    x.block(0, 0, 3*N, 1) = (1.0-t)*x0.block(0, 0, 3*N, 1) + (t)*x1.block(0, 0, 3*N, 1);
    // interpolate rotation with quaternions slerp
    for (int n = 0; n < N; n++) {
        double theta0 = x0.segment<3>(3 * N + 3 * n).norm();
        Vector3s v0;
        if (theta0 > 0.0)
            v0 = x0.segment<3>(3 * N + 3 * n) / theta0;
        else
            v0 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa0(theta0, v0);
        Eigen::Quaterniond quat0(aa0);
        
        double theta1 = x1.segment<3>(3 * N + 3 * n).norm();
        Vector3s v1;
        if (theta1 > 0.0)
            v1 = x1.segment<3>(3 * N + 3 * n) / theta1;
        else
            v1 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa1(theta1, v1);
        Eigen::Quaterniond quat1(aa1);
        
        // Use Quaternion SLERP for rotation interpolation
        Eigen::Quaterniond quat = quat0.slerp(t, quat1);
        
        // convert back to axis-angle representation
        Eigen::AngleAxisd aa(quat);
        x.segment<3>(3 * N + 3 * n) = aa.angle() * aa.axis();
    }
    
    return x;
}

VectorXs SCISim::get_dxdt_from_x(const VectorXs& x0, const VectorXs& x1,
                                    double h) {
    
    if (x0.rows() != x1.rows())
        throw "x0 and x1 must be the same size.";
    
    if (x0.rows() % 6)
        throw "x0 and x1 must have size of 6*N for N objects.";
    
    int N = x0.rows() / 6;
    
    if (h <= 0.0)
        throw "h must be bigger then 0";
    
    VectorXs v = VectorXs::Zero(N*6);
    
    // interpolate translation
    v.block(0, 0, 3*N, 1) = (x1.block(0, 0, 3*N, 1) - x0.block(0, 0, 3*N, 1)) / h;
    // interpolate rotation
    for (int n = 0; n < N; n++) {
        double theta0 = x0.segment<3>(3 * N + 3 * n).norm();
        Vector3s v0;
        if (theta0 > 0.0)
            v0 = x0.segment<3>(3 * N + 3 * n) / theta0;
        else
            v0 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa0(theta0, v0);
        Eigen::Matrix3d R0;
        R0 = aa0;
        
        double theta1 = x1.segment<3>(3 * N + 3 * n).norm();
        Vector3s v1;
        if (theta1 > 0.0)
            v1 = x1.segment<3>(3 * N + 3 * n) / theta1;
        else
            v1 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa1(theta1, v1);
        Eigen::Matrix3d R1;
        R1 = aa1;
        
        Eigen::Matrix3d R = R1*(R0.inverse());
        Eigen::Matrix3d log_R = R.log();
        
        // Store angular velocity
        v(3 * N + 3 * n + 0) = log_R(2, 1) / h;
        v(3 * N + 3 * n + 1) = log_R(0, 2) / h;
        v(3 * N + 3 * n + 2) = log_R(1, 0) / h;
    }
    
    return v;
}

VectorXs SCISim::get_x1_from_x0_dxdt(const VectorXs& x0,
                                        const VectorXs& v, double h) {
    if (x0.rows() != v.rows())
        throw "x0 and v must be the same size.";
    
    if (x0.rows() % 6)
        throw "x0 and v must have size of 6*N for N objects.";
    
    int N = x0.rows() / 6;
    
    if (h <= 0.0)
        throw "h must be bigger then 0";
    
    VectorXs x1 = VectorXs::Zero(N*6);
    
    // calculate translation
    x1.block(0, 0, 3*N, 1) = x0.block(0, 0, 3*N, 1) + v.block(0, 0, 3*N, 1) * h;
    // calculate rotation
    for (int n = 0; n < N; n++) {
        double theta0 = x0.segment<3>(3 * N + 3 * n).norm();
        Vector3s v0;
        if (theta0 > 0.0)
            v0 = x0.segment<3>(3 * N + 3 * n) / theta0;
        else
            v0 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa0(theta0, v0);
        Eigen::Matrix3d R0;
        R0 = aa0;
        
        Eigen::Matrix3d log_R = Eigen::Matrix3d::Zero();
        
        // Load angular velocity
        log_R(2, 1) = v(3 * N + 3 * n + 0);
        log_R(1, 2) = -v(3 * N + 3 * n + 0);
        log_R(0, 2) = v(3 * N + 3 * n + 1);
        log_R(2, 0) = -v(3 * N + 3 * n + 1);
        log_R(1, 0) = v(3 * N + 3 * n + 2);
        log_R(0, 1) = -v(3 * N + 3 * n + 2);
        
        // apply angular velocity rotation to x0
        Eigen::Matrix3d R = (log_R * h).exp();
        Eigen::Matrix3d R1 = R*R0;
        
        // convert x1 rotation to angle-axis
        Eigen::AngleAxisd aa1;
        aa1 = R1;
        
        // store x1
        x1.segment<3>(3 * N + 3 * n) = aa1.axis() * aa1.angle();
    }
    
    return v;
}

VectorXs SCISim::get_x() {
    // read current configuration state (translation and rotation matrix)
    return get_x_from_q(getSimState_q());
}


VectorXs SCISim::get_v() {
    return getSimState_v();
}

void SCISim::set_x(const VectorXs& x) {
    setSimState_q(get_q_from_x(x));
}

void SCISim::set_v(const VectorXs& dxdt) {
    setSimState_v(dxdt);
}

void SCISim::set_dxdt(const VectorXs& x0, const VectorXs& x1, double h) {
    VectorXs dxdt = get_dxdt_from_x(x0, x1, h);
    setSimState_v(dxdt);
}

void SCISim::get_contacts_normal_and_body_ind( ContactNormalVec& normal_and_body_ind_vec )
{
    const RigidBodySimState& m_sim_state = m_sim.getState();
    VectorXs contact_normal;
    VectorXs q;
    
    normal_and_body_ind_vec.clear();
    
    std::vector<std::unique_ptr<Constraint>> active_set;
    m_sim.computeActiveSet( m_sim_state.q(), m_sim_state.q(), active_set );
    //for( std::vector<std::unique_ptr<Constraint>>::size_type i = 0; i < active_set.size(); ++i )
    for( const std::unique_ptr<Constraint>& constraint : active_set )
    {
        if( constraint->getName() == "sphere_sphere" )
        {
            SphereSphereConstraint& sphere_sphere = sd_cast< SphereSphereConstraint&>( *constraint );
            sphere_sphere.getWorldSpaceContactNormal( q, contact_normal );
            normal_and_body_ind_vec.push_back(ContactNormalElement(
                                                                   contact_normal, sphere_sphere.idx1()));
        }
        else if( constraint->getName() == "teleported_sphere_sphere" )
        {
            // TODO: Clean this up!
            TeleportedSphereSphereConstraint& teleported_sphere_sphere = sd_cast< TeleportedSphereSphereConstraint&>( *constraint );
            teleported_sphere_sphere.getWorldSpaceContactNormal( q, contact_normal );
            normal_and_body_ind_vec.push_back(ContactNormalElement(
                                                                   contact_normal, teleported_sphere_sphere.idx1()));
        }
        else if( constraint->getName() == "static_plane_sphere" )
        {
            StaticPlaneSphereConstraint& plane_sphere = sd_cast< StaticPlaneSphereConstraint&>( *constraint );
            plane_sphere.getWorldSpaceContactNormal( q, contact_normal );
            normal_and_body_ind_vec.push_back(ContactNormalElement(
                                                                   contact_normal, plane_sphere.sphereIdx()));
        }
        else
        {
            std::cerr << "Warning, penetration depth computation not implemented for: " << constraint->getName() << std::endl;
        }
    }
}

void SCISim::resolve_contact() {
    ContactNormalVec normal_and_body_ind_vec;
    bool is_contact;
    
    const RigidBodySimState& m_sim_state = m_sim.getState();
    VectorXs& q = const_cast< VectorXs& >(m_sim_state.q());
    
    // as long as there is contact, push bodies away
    do {
        //        cout<<"HERE"<<endl;
        get_contacts_normal_and_body_ind(normal_and_body_ind_vec);
        if (normal_and_body_ind_vec.size())
            is_contact = true;
        else
            is_contact = false;
        
        // loop over all contacts and push objects in direction of normal
        for( ContactNormalElement normal_and_body_ind : normal_and_body_ind_vec ) {
            // update translation by 1 centimeter
            q.segment<3>(3 * normal_and_body_ind.second) += normal_and_body_ind.first*0.01;
            //            cout<<"Ind: "<<normal_and_body_ind.second<<"    n: "<<normal_and_body_ind.first(0)<<", "<<normal_and_body_ind.first(1)<<", "<<normal_and_body_ind.first(2)<<endl;
        }
    } while (is_contact);
}


////////////////////////////////////////////
// SCISimApp
////////////////////////////////////////////
int SCISimApp::Qt_argc = 1;
char *SCISimApp::Qt_argv[] = { "PySCISim" };

SCISimApp::SCISimApp(bool process_Qt_events) :
		process_Qt_events(process_Qt_events) {
	collision_penetration_depth = 0.0;

	app = NULL;
	window = NULL;

	get_gl_widget();
	updateSimData();
}

SCISimApp::~SCISimApp() {
	resetQt();
}

void SCISimApp::resetQt() {
	if (app) {
		app->quit();
		delete app;
	}
//	if (window)
//		delete window;

	//QApplication::setGraphicsSystem("opengl");
	app = new QApplication(Qt_argc, Qt_argv);
	window = new Window;
}

void SCISimApp::centerWindow() {
	QDesktopWidget* desktop = QApplication::desktop();

	const int screenWidth = desktop->screenGeometry().width();
	const int screenHeight = desktop->screenGeometry().height();

	const QSize windowSize = window->size();
	const int width = windowSize.width();
	const int height = windowSize.height();

	const int x = (screenWidth - width) / 2;
	const int y = (screenHeight - height) / 2;

	window->move(x, y);
}

GLWidget* SCISimApp::get_gl_widget() {
	// Make sure old window is destroyed
	resetQt();

	m_gl_widget = window->get_content_widget()->get_gl_widget();
	m_gl_widget->useOpenGL(process_Qt_events);

	if (!process_Qt_events) {
		window->hide();
		m_gl_widget->toggleHUD();
	} else {
		window->resize(window->sizeHint());
		window->setWindowTitle("Three Dimensional Rigid Body Simulation");
		//	centerWindow();
		window->show();
		window->raise();
	}

	app->processEvents();
	return m_gl_widget;
}

void SCISimApp::run(const std::string& xml_scene_file_name) {
    // load a scene if given
    if (xml_scene_file_name != "") {
        openScene(xml_scene_file_name);
    }
    
    app->exec();
    
    // reload scene/clear window or else Qt crashes.
    if (xml_scene_file_name != "") {
        openScene(xml_scene_file_name);
    } else
        // store new gl_widget
        get_gl_widget();
}

void SCISimApp::openScene(const std::string& xml_scene_file_name, unsigned fps, bool render_at_fps, bool lock_camera) {
	get_gl_widget();
	m_gl_widget->openScene(xml_scene_file_name.c_str(), fps, render_at_fps, lock_camera);
	updateSimData();
}

void SCISimApp::stepSystem() {
	// step simulation
	m_gl_widget->stepSystem();
	// read simulation variables
	updateSimData();
	// step Qt events loop
	if (process_Qt_events)
		app->processEvents();
}

void SCISimApp::resetSystem() {
	m_gl_widget->resetSystem();
}

void SCISimApp::updateSimData() {
	m_gl_widget->getSimData(time, T, U, p, L);
	m_gl_widget->get_sim()->computeNumberOfCollisions( number_of_collisions, collision_penetration_depth );
	// FIXME: the next function generates warnings that crash ParallelPython.
//	m_gl_widget->get_sim()->computeNumberOfCollisions(number_of_collisions,
//			collision_penetration_depth);
}

double SCISimApp::getSim_time() {
	return time;
}

double SCISimApp::getSim_T() {
	return T;
}

double SCISimApp::getSim_U() {
	return U;
}

Eigen::Vector3d SCISimApp::getSim_p() {
	return p;
}

Eigen::Vector3d SCISimApp::getSim_L() {
	return L;
}

unsigned int SCISimApp::getSim_collisions() {
	return number_of_collisions;
}

double SCISimApp::getSim_collisions_penetration_depth() {
	return collision_penetration_depth;
}

ThreeDRigidBodySim* SCISimApp::getSim_sim() {
	return m_gl_widget->get_sim();
}

RigidBodySimState* SCISimApp::getSim_sim_state_backup() {
	return m_gl_widget->get_sim_state_backup();
}

MatrixXs SCISimApp::getSimState_M() {
	return getSim_sim()->getState().M();
}

VectorXs SCISimApp::getSimState_q() {
	return getSim_sim()->getState().q();
}

VectorXs SCISimApp::getSimState_v() {
	return getSim_sim()->getState().v();
}

VectorXs SCISimApp::getSimState_T() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	int N = m_sim_state.nbodies();
	VectorXs T(N);

	VectorXs T_helper = 0.5
			* m_sim_state.v().cwiseProduct(m_sim_state.M() * m_sim_state.v());

	// sum angular and linear kinetic energy
	for (int body = 0; body < N; ++body) {
		T(body) = T_helper.segment<3>(3 * body).sum()
				+ T_helper.segment<3>(3 * N + 3 * body).sum();
	}

	return T;
}

VectorXs SCISimApp::getSimState_U() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	int N = m_sim_state.nbodies();
	VectorXs U = VectorXs::Zero(N);

	for (int body = 0; body < N; ++body) {
		VectorXs cur_q(12);
		cur_q.segment<3>(0) = m_sim_state.q().segment<3>(3*body);
		cur_q.segment<9>(3) = m_sim_state.q().segment<9>(3*N+9*body);
		MatrixXs M = this->getSimState_M();
		SparseMatrixsc cur_M(6, 6);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) {
				cur_M.insert(i, j) = M(3*body+i, 3*body+j);
				cur_M.insert(i+3, j+3) = M(3*N+3*body+i, 3*N+3*body+j);
			}
		for (std::vector<Force*>::size_type i = 0; i < m_sim_state.forces().size();
				++i) {
			U(body) += m_sim_state.forces()[i]->computePotential(
					cur_q, cur_M);
		}
	}

	return U;
}

VectorXs SCISimApp::getSimState_p() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	int N = m_sim_state.nbodies();
	VectorXs p(N * 3);

	for (int body = 0; body < N; ++body) {
		p.segment<3>(3 * body) = m_sim_state.getTotalMass(body)
				* m_sim_state.v().segment<3>(3 * body);
	}

	return p;
}

VectorXs SCISimApp::getSimState_L() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	int N = m_sim_state.nbodies();
	VectorXs L = VectorXs::Zero(N * 3);

	// Contribution from center of mass
	for (int body = 0; body < m_sim_state.nbodies(); ++body) {
		L.segment<3>(3 * body) += m_sim_state.getTotalMass(body)
				* m_sim_state.q().segment<3>(3 * body).cross(
						m_sim_state.v().segment<3>(3 * body));
	}

	// Contribution from rotation about center of mass
	for (int body = 0; body < m_sim_state.nbodies(); ++body) {
		L.segment<3>(3 * body) += m_sim_state.getInertia(body)
				* m_sim_state.v().segment<3>(
						3 * m_sim_state.nbodies() + 3 * body);
	}

	return L;
}

int SCISimApp::getSimState_nbodies() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	return m_sim_state.nbodies();
}

void SCISimApp::setSimState_q(const VectorXs& q) {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	VectorXs& cur_q = const_cast< VectorXs& >(m_sim_state.q());

	if ((q.cols() != cur_q.cols()) || (q.rows() != cur_q.rows()))
		throw "Wrong dimensions of q";

	cur_q = q;
}

void SCISimApp::setSimState_v(const VectorXs& v) {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	VectorXs& cur_v = const_cast< VectorXs& >(m_sim_state.v());

	if ((v.cols() != cur_v.cols()) || (v.rows() != cur_v.rows()))
		throw "Wrong dimensions of v";

	cur_v = v;
}

VectorXs SCISimApp::get_x_from_q(const VectorXs& q) {
	int N = this->getSimState_nbodies();
	VectorXs x = VectorXs::Zero(N*6);

	// copy translation as is
	x.block(0, 0, 3*N, 1) = q.block(0, 0, 3*N, 1);
	// convert rotation matrix into axis/angle
	for (int n = 0; n < N; n++) {
		const Matrix33sr R = Eigen::Map<const Matrix33sr>( q.segment<9>( 3 * N + 9 * n ).data() );
		Matrix33sr log_R = R.log();
		x(3 * N + 3 * n + 0, 0) = log_R(2, 1);
		x(3 * N + 3 * n + 1, 0) = log_R(0, 2);
		x(3 * N + 3 * n + 2, 0) = log_R(1, 0);
	}

	return x;
}

// converts
VectorXs SCISimApp::get_q_from_x(const VectorXs& x) {
	int N = this->getSimState_nbodies();
	// check for correct input size
	if (x.rows() != N*6)
		throw "Wrong dimensions of x";

	VectorXs q = VectorXs::Zero(N*12);

	// copy translation as is
	q.block(0, 0, 3*N, 1) = x.block(0, 0, 3*N, 1);
	// convert rotation matrix into axis/angle
	for (int n = 0; n < N; n++) {
		Matrix33sr log_R = Matrix33sr::Zero();

		log_R(0, 1) = -x(3 * N + 3 * n + 2, 0);
		log_R(0, 2) = x(3 * N + 3 * n + 1, 0);
		log_R(1, 0) = x(3 * N + 3 * n + 2, 0);
		log_R(1, 2) = -x(3 * N + 3 * n + 0, 0);
		log_R(2, 0) = -x(3 * N + 3 * n + 1, 0);
		log_R(2, 1) = x(3 * N + 3 * n + 0, 0);

		Matrix33sr R = log_R.exp();
        q.segment<9>( 3 * N + 9 * n ) = Eigen::Map< VectorXs >(R.data(), R.size());
	}

	return q;
}

VectorXs SCISimApp::interpolate_x(const VectorXs& x0, const VectorXs& x1, scalar t) {
	if (t <= 0.0)
		return x0;
	if (t >= 1.0)
		return x1;

	int N = this->getSimState_nbodies();
	VectorXs x = VectorXs::Zero(N*6);

	// interpolate translation
	x.block(0, 0, 3*N, 1) = (1.0-t)*x0.block(0, 0, 3*N, 1) + (t)*x1.block(0, 0, 3*N, 1);
	// interpolate rotation with quaternions slerp
	for (int n = 0; n < N; n++) {
		double theta0 = x0.segment<3>(3 * N + 3 * n).norm();
		Vector3s v0;
		if (theta0 > 0.0)
			v0 = x0.segment<3>(3 * N + 3 * n) / theta0;
		else
			v0 << 1.0, 0.0, 0.0;

        Eigen::AngleAxisd aa0(theta0, v0);
        Eigen::Quaterniond quat0(aa0);

        double theta1 = x1.segment<3>(3 * N + 3 * n).norm();
        Vector3s v1;
        if (theta1 > 0.0)
            v1 = x1.segment<3>(3 * N + 3 * n) / theta1;
        else
            v1 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa1(theta1, v1);
        Eigen::Quaterniond quat1(aa1);

        // Use Quaternion SLERP for rotation interpolation
        Eigen::Quaterniond quat = quat0.slerp(t, quat1);
        
        // convert back to axis-angle representation
        Eigen::AngleAxisd aa(quat);
        x.segment<3>(3 * N + 3 * n) = aa.angle() * aa.axis();
    }
    
    return x;
}

VectorXs SCISimApp::get_dxdt_from_x(const VectorXs& x0, const VectorXs& x1,
                                    double h) {
  
    if (x0.rows() != x1.rows())
        throw "x0 and x1 must be the same size.";
    
    if (x0.rows() % 6)
        throw "x0 and x1 must have size of 6*N for N objects.";
        
    int N = x0.rows() / 6;
    
    if (h <= 0.0)
        throw "h must be bigger then 0";

    VectorXs v = VectorXs::Zero(N*6);
    
    // interpolate translation
    v.block(0, 0, 3*N, 1) = (x1.block(0, 0, 3*N, 1) - x0.block(0, 0, 3*N, 1)) / h;
    // interpolate rotation
    for (int n = 0; n < N; n++) {
        double theta0 = x0.segment<3>(3 * N + 3 * n).norm();
        Vector3s v0;
        if (theta0 > 0.0)
            v0 = x0.segment<3>(3 * N + 3 * n) / theta0;
        else
            v0 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa0(theta0, v0);
        Eigen::Matrix3d R0;
        R0 = aa0;
        
        double theta1 = x1.segment<3>(3 * N + 3 * n).norm();
        Vector3s v1;
        if (theta1 > 0.0)
            v1 = x1.segment<3>(3 * N + 3 * n) / theta1;
        else
            v1 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa1(theta1, v1);
        Eigen::Matrix3d R1;
        R1 = aa1;
        
        Eigen::Matrix3d R = R1*(R0.inverse());
        Eigen::Matrix3d log_R = R.log();
        
        // Store angular velocity
        v(3 * N + 3 * n + 0) = log_R(2, 1) / h;
        v(3 * N + 3 * n + 1) = log_R(0, 2) / h;
        v(3 * N + 3 * n + 2) = log_R(1, 0) / h;
    }
    
    return v;
}

VectorXs SCISimApp::get_x1_from_x0_dxdt(const VectorXs& x0,
                                        const VectorXs& v, double h) {
    if (x0.rows() != v.rows())
        throw "x0 and v must be the same size.";
    
    if (x0.rows() % 6)
        throw "x0 and v must have size of 6*N for N objects.";
    
    int N = x0.rows() / 6;
    
    if (h <= 0.0)
        throw "h must be bigger then 0";
    
    VectorXs x1 = VectorXs::Zero(N*6);
    
    // calculate translation
    x1.block(0, 0, 3*N, 1) = x0.block(0, 0, 3*N, 1) + v.block(0, 0, 3*N, 1) * h;
    // calculate rotation
    for (int n = 0; n < N; n++) {
        double theta0 = x0.segment<3>(3 * N + 3 * n).norm();
        Vector3s v0;
        if (theta0 > 0.0)
            v0 = x0.segment<3>(3 * N + 3 * n) / theta0;
        else
            v0 << 1.0, 0.0, 0.0;
        
        Eigen::AngleAxisd aa0(theta0, v0);
        Eigen::Matrix3d R0;
        R0 = aa0;
        
        Eigen::Matrix3d log_R = Eigen::Matrix3d::Zero();

        // Load angular velocity
        log_R(2, 1) = v(3 * N + 3 * n + 0);
        log_R(1, 2) = -v(3 * N + 3 * n + 0);
        log_R(0, 2) = v(3 * N + 3 * n + 1);
        log_R(2, 0) = -v(3 * N + 3 * n + 1);
        log_R(1, 0) = v(3 * N + 3 * n + 2);
        log_R(0, 1) = -v(3 * N + 3 * n + 2);
        
        // apply angular velocity rotation to x0
        Eigen::Matrix3d R = (log_R * h).exp();
        Eigen::Matrix3d R1 = R*R0;
        
        // convert x1 rotation to angle-axis
        Eigen::AngleAxisd aa1;
        aa1 = R1;
        
        // store x1
        x1.segment<3>(3 * N + 3 * n) = aa1.axis() * aa1.angle();
    }
    
    return v;
}

VectorXs SCISimApp::get_x() {
	// read current configuration state (translation and rotation matrix)
	return get_x_from_q(getSimState_q());
}


VectorXs SCISimApp::get_v() {
	return getSimState_v();
}

void SCISimApp::set_x(const VectorXs& x) {
	setSimState_q(get_q_from_x(x));
}

void SCISimApp::set_v(const VectorXs& dxdt) {
	setSimState_v(dxdt);
}

void SCISimApp::set_dxdt(const VectorXs& x0, const VectorXs& x1, double h) {
    VectorXs dxdt = get_dxdt_from_x(x0, x1, h);
    setSimState_v(dxdt);
}

void SCISimApp::get_contacts_normal_and_body_ind( ContactNormalVec& normal_and_body_ind_vec )
{
    const RigidBodySimState& m_sim_state = getSim_sim()->getState();
    VectorXs contact_normal;
    VectorXs q;

    normal_and_body_ind_vec.clear();
    
    std::vector<std::unique_ptr<Constraint>> active_set;
    m_gl_widget->get_sim()->computeActiveSet( m_sim_state.q(), m_sim_state.q(), active_set );
    //for( std::vector<std::unique_ptr<Constraint>>::size_type i = 0; i < active_set.size(); ++i )
    for( const std::unique_ptr<Constraint>& constraint : active_set )
    {
        if( constraint->getName() == "sphere_sphere" )
        {
            SphereSphereConstraint& sphere_sphere = sd_cast< SphereSphereConstraint&>( *constraint );
            sphere_sphere.getWorldSpaceContactNormal( q, contact_normal );
            normal_and_body_ind_vec.push_back(ContactNormalElement(
                            contact_normal, sphere_sphere.idx1()));
        }
        else if( constraint->getName() == "teleported_sphere_sphere" )
        {
            // TODO: Clean this up!
            TeleportedSphereSphereConstraint& teleported_sphere_sphere = sd_cast< TeleportedSphereSphereConstraint&>( *constraint );
            teleported_sphere_sphere.getWorldSpaceContactNormal( q, contact_normal );
            normal_and_body_ind_vec.push_back(ContactNormalElement(
                   contact_normal, teleported_sphere_sphere.idx1()));
        }
        else if( constraint->getName() == "static_plane_sphere" )
        {
            StaticPlaneSphereConstraint& plane_sphere = sd_cast< StaticPlaneSphereConstraint&>( *constraint );
            plane_sphere.getWorldSpaceContactNormal( q, contact_normal );
            normal_and_body_ind_vec.push_back(ContactNormalElement(
                   contact_normal, plane_sphere.sphereIdx()));
        }
        else
        {
            std::cerr << "Warning, penetration depth computation not implemented for: " << constraint->getName() << std::endl;
        }
    }
}

void SCISimApp::resolve_contact() {
    ContactNormalVec normal_and_body_ind_vec;
    bool is_contact;

    const RigidBodySimState& m_sim_state = getSim_sim()->getState();
    VectorXs& q = const_cast< VectorXs& >(m_sim_state.q());
    
    // as long as there is contact, push bodies away
    do {
//        cout<<"HERE"<<endl;
        get_contacts_normal_and_body_ind(normal_and_body_ind_vec);
        if (normal_and_body_ind_vec.size())
            is_contact = true;
        else
            is_contact = false;
        
        // loop over all contacts and push objects in direction of normal
        for( ContactNormalElement normal_and_body_ind : normal_and_body_ind_vec ) {
            // update translation by 1 centimeter
            q.segment<3>(3 * normal_and_body_ind.second) += normal_and_body_ind.first*0.01;
//            cout<<"Ind: "<<normal_and_body_ind.second<<"    n: "<<normal_and_body_ind.first(0)<<", "<<normal_and_body_ind.first(1)<<", "<<normal_and_body_ind.first(2)<<endl;
        }
    } while (is_contact);
}

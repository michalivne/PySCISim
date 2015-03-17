#include "../include/SimulationSCISIM.h"

SimulationSCISIM::SimulationSCISIM(const DesignState &designState, double restitution, double friction, unsigned int staticSceneId, bool computeGradients, double dt)
{
    m_unconstrained_map = NULL;
    m_impact_operator = NULL;
    m_friction_operator = NULL;
    m_impact_friction_map = NULL;
    m_scripting_callback = NULL;
    
    m_solvers.push_back("ma57");
    m_numSCISIMObj = 0;
    m_computeGradients = computeGradients;
    m_dt = dt;
    m_CoR = restitution;
    m_mu = friction;
    loadScene(designState, staticSceneId);

}

SimulationSCISIM::SimulationSCISIM(const SimulationSCISIM &toCopy)
{
    std::cout<<"SimulationSCISIM is not copyable\n";
    exit(0);
}

SimulationSCISIM::~SimulationSCISIM()
{
    
}

void SimulationSCISIM::operator=(const SimulationSCISIM &toAssign)
{
    std::cout<<"SimulationSCISIM is not assignable\n";
    exit(0);
}

void SimulationSCISIM::initStaticScene(unsigned int sceneId)
{
    
    m_sceneId = sceneId;
    //SCISIM doesn't use namespaces so have to specify global namespace ... sigh ...
    
    //clear all static objects in scene and add in the correct ones relating to the starting criteris
    switch(sceneId) {
        case 0:
            break; //do nothing
        case 1:
            //a single plane with normal (0,1,0)
            m_sim_state.addStaticPlane(::StaticPlane(Vector3s(0,0,0), Vector3s(0,1,0)));
            m_designState.addStaticPlane(Vector3s(0,0,0), Vector3s(0,1,0));
            break;
        case 2:
            //our experimental ramp setup.
            //ramp is 15.9 degrees
            //ramp hits horizontal plane at (0,0)
            m_sim_state.addStaticPlane(::StaticPlane(Vector3s(0,0,0), Vector3s(0,1,0)));
            m_designState.addStaticPlane(Vector3s(0,0,0), Vector3s(0,1,0));
            
            //our ramp
            m_sim_state.addStaticPlane(::StaticPlane(Vector3s(-0.6096,0.1737,0), Vector3s(0.2741, 0.9617,0))); //plane at 15.9 degrees
            m_designState.addStaticPlane(Vector3s(-0.6096,0.1737,0), Vector3s(0.2741, 0.9617,0));
            
            break;
        case 3:
            //our ramp
            m_sim_state.addStaticPlane(::StaticPlane(Vector3s(-0.6096,0.1737,0), Vector3s(0.2741, 0.9617,0))); //plane at 15.9 degrees
            m_designState.addStaticPlane(Vector3s(-0.6096,0.1737,0), Vector3s(0.2741, 0.9617,0));

            break;
        default:
            std::cout<<"SimulationSCISIM::initStaticScene: unknown scene id "<<sceneId<<"\n";
    };
}

void SimulationSCISIM::loadScene(const DesignState designState, unsigned int staticSceneId)
{
    
    Matrix33sr R0;
    Vector3s x, v, omega;
    VectorXs Rmat;
    double rho;
    Mesh mesh;
    RigidBodyGeometry *bodyGeometry;
    
    //Temporatry storage for mesh verts / faces
    Matrix3Xuc faces;
    Matrix3Xsc vertices;
    
    //The things that SCISIM computes
    scalar M;
    Vector3s CM;
    Vector3s I;
    
    //Dave - Density hardcoded for 3D printing (for now)
    rho = 1005.8; //value for stratysys 3D printers
    
    //clear scisim state
    if(m_unconstrained_map)
    {
        delete m_unconstrained_map;
        delete m_impact_operator;
        delete m_friction_operator;
        delete m_impact_friction_map;
        delete m_scripting_callback;
        
        //delete m_trajectoryModel;
        m_sim_state = RigidBodySimState();
        m_xs.clear();
        m_vs.clear();
        m_omegas.clear();
        m_Rs.clear();
        m_Ms.clear(); //Linear inertia
        m_I0s.clear(); //Rotational inertia
        m_numSCISIMObj = 0;
        m_R0.clear();
        m_COM.clear();
        m_geometry.clear();
        m_geometryIndices.clear();
        m_fixed.clear();
        
        m_xs_desdyn_params.clear();
        m_vs_desdyn_params.clear();
        m_Rs_desdyn_params.clear();
        m_omegas_desdyn_params.clear();
        m_designToSCISIM.clear();
        m_staticPlanes.clear();
        m_designToSCISIM.clear();
        m_shape.clear();
        m_shape_desdyn_params.clear();
     
    }
    
    //Setup SCISIM Integrators and other miscellaneous crap
    // Set flow maps:
    m_unconstrained_map = new DMVMap(); // Use DMV as the Unconstrained Map
    //m_impact_friction_map = new StabilizedImpactFrictionMap( 1e-6 , 1e-6 , 10000 );
		//m_impact_friction_map = new GeometricImpactFrictionMap( 1e-6 , 1e-6 , 1000000 );
    m_impact_friction_map = new GeometricImpactFrictionMap( 1e-4 , 1e-4 , 50, false, false );

    
    // Set contact operators:
    //m_impact_operator = new LCPOperatorQL( 1e-8); //QL for now
    //m_friction_operator = new LinearMDPOperatorQL( 4 , 1e-8 ); //QL for now
    
    m_impact_operator = new LCPOperatorIpopt(m_solvers, 1e-9); //QL for now
    m_friction_operator = new LinearMDPOperatorIpopt( 4 , m_solvers, 1e-9); //QL for now
   
    // We don't need no stinkin' callback :) --  yet ...
    m_scripting_callback = new RigidBodyScriptingCallback("nullscripting");

    
    //load everything into SCISIM and we're good to go!!
    m_sim_state.setState(m_xs, m_vs, m_Ms, m_Rs, m_omegas, m_I0s, m_fixed, m_geometryIndices, m_geometry);
	
    m_sim_state.addForce(new NearEarthGravityForce(Vector3s(0.0, -9.8, 0.0)));
	
    initStaticScene(staticSceneId); //ramps and othe things we need
    
    m_sim.setState(m_sim_state);
		fixInitialConditions();


}

void SimulationSCISIM::runTo(double time, bool timeCheck)
{
  
	//Check simulation time
	StopWatch stopWatch;
	m_iteration = 0;
	m_t = 0.0;
	
	//Get to integrating!
	if(timeCheck)
	stopWatch.start();
	
		while(m_t < time)
		{
			// TODO(DK): put back constrained flow, later have scene file specify integration method as in SCISIM files
			m_iteration++;
			// m_sim.flow( next_iter, m_dt, *m_unconstrained_map);
			// m_sim.flow( m_iteration, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR);
			m_sim.flow( *m_scripting_callback ,m_iteration , m_dt , *m_unconstrained_map , *m_impact_operator , m_CoR , *m_friction_operator, m_mu , *m_impact_friction_map);
			
			m_t += m_dt;
		}
		
	}
}

double SimulationSCISIM::takeStep()
{
    // TODO(DK): put back constrained flow, later have scene file specify integration method as in SCISIM files
    m_iteration++;
    m_t += m_dt;
    
    if(m_computeGradients) {
        
        // m_sim.flow( nextIteration, dt, *m_unconstrained_map);
        // m_sim.flow( nextIteration, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR);
        m_sim.flow(*m_scripting_callback,m_iteration, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR, *m_friction_operator, m_mu,*m_impact_friction_map);
      

    } else {
        // m_sim.flow( nextIteration, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR);
        m_sim.flow(*m_scripting_callback,m_iteration, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR, *m_friction_operator, m_mu,*m_impact_friction_map);

    }
    

    
    return m_dt;
}


void SimulationSCISIM::setDt(double dt) {
    m_dt = dt;
}


Matrix33sr SimulationSCISIM::rigidBodyRotation(unsigned int objIndex) const
{
 
    //correct for SCISIM rotation
    
    return Eigen::Map<const Matrix33sr>(m_sim.getState().q().segment<9>(3*m_sim_state.nbodies() + 9*m_designToSCISIM[objIndex].scisimId()).data())*m_R0[objIndex].transpose();
    
}

Vector3s SimulationSCISIM::rigidBodyPosition(unsigned int objIndex) const
{
    //take into account
   return (m_sim.getState().q().segment<3>(3*m_designToSCISIM[objIndex].scisimId()) + m_COM[objIndex]);
}


void SimulationSCISIM::setObjectParameters(unsigned int objectId, const double *params, unsigned int size)
{
    m_designState.setObjectParameters(objectId, params, size);
    initSCISIMFromDesign();
}

void SimulationSCISIM::setObjectParameters(const double *params)
{
    m_designState.setObjectParameters(params);
    initSCISIMFromDesign();
}

void SimulationSCISIM::fixInitialConditions()
{

    Matrix33sr R0; //actual rotation to reset to
    validInitialConditions();
    m_iteration = 0;
    m_t = 0.0;
    for(unsigned int i=0; i<m_designState.numObjects(); ++i)
    {
        R0 = Eigen::Map<const Matrix33sr>(m_designState.rotation(i))*m_R0[i];

        const_cast<RigidBodySimState &>(m_sim.getState()).q().segment<9>(3*m_sim_state.nbodies() + 9*m_designToSCISIM[i].scisimId()) = Eigen::Map<const VecXd>(R0.data(),9);

        //take into account center of mass
        const_cast<RigidBodySimState &>(m_sim.getState()).q().segment<3>(3*m_designToSCISIM[i].scisimId()) = Eigen::Map<const VecXd>(m_designState.position(i),3)+ m_COM[i];

        //std::cout<<"Position: \n"<<Eigen::Map<const VecXd>(m_designState.position(i),3)<<"\n";
    }
    
    const_cast<RigidBodySimState &>(m_sim.getState()).v().setZero();
}

void SimulationSCISIM::validInitialConditions()
{
    //All geometry is triangle meshes here
    RigidBodyTriangleMesh *geo;
    std::vector<FeasibilityConstraint> constraints;
    FeasibilityConstraint con;
    MatXd G;
    VecXd g0;
    MatXd CI;
    VecXd ci;
    unsigned int n = designState().numObjects()*3;
    unsigned int m;
    
    
    //find vertices with constraint violation, add constraints, move center initial positions to correct
    //x = [p1x p1y p1z .... pnx pny pnz] where pi is initial center of mass positon for ith object
    for(unsigned int obj = 0; obj < designState().numObjects(); ++obj)
    {
        geo = dynamic_cast<RigidBodyTriangleMesh *>(m_sim.state().geometry()[obj]);
        
        Vec3d pos(designState().position(obj)); //take into acount center of mass
        Matrix33sr rot(designState().rotation(obj));
        
        //modify rotation to account for SCISIM change
        rot = rot*m_R0[obj];
        for(unsigned int v=0; v<geo->vertices().cols(); ++v)
        {
            Vec3d vertex = geo->vertices().block(0, v, 3, 1);
            
            for(unsigned int p=0; p<m_sim.state().staticPlanes().size(); ++p)
            {
                const ::StaticPlane &plane = m_sim.state().staticPlane(p);
                
                if(plane.n().transpose().dot(pos + m_COM[obj] + rot*vertex - plane.x()) < 0) {
                    con.n = plane.n();
                    con.rhs = con.n.transpose().dot(m_COM[obj] + rot*vertex - plane.x()) - 1e-4;
                    con.objectId = obj;
                    constraints.push_back(con);
                }
            }
        }
        
    }
    
    m = constraints.size();
    
    if(m > 0) {
        //setup embarassingly simple QP, hessian is identity (could be mass weighted or something but screw it for now)
        //setup constraint matrix
        //solve using Quadprog++
        G.resize(n,n);
        G.setIdentity();
        g0.resize(n);
        g0 = Eigen::Map<const VecXd>(designState().position(0), n);
        g0 = -g0;
        
        //build the motherf'n constraint matrix and vector then solve this mutha!
        CI.resize(m,n);
        CI.setZero();
        ci.resize(m);
        ci.setZero();
        
        for(unsigned int c=0; c<m; ++c)
        {

            CI.block(c,3*constraints[c].objectId,1,3) = constraints[c].n.transpose();
            
            //constraint matrix entry is just a scatter of normal components
            ci(c) = constraints[c].rhs;
        }
        
        MatXd CE;
        VecXd ce0, x;
        
        CE.resize(0,n);
        ce0.resize(0);
        
        Eigen::solve_quadprog(G, g0, CE.transpose(), ce0, CI.transpose(), ci, x);
        
        //Copy
        memcpy(designState().position(0), x.data(), sizeof(double)*n);
    }
    
}

void SimulationSCISIM::reloadScene()
{
    loadScene(m_designState, staticSceneId());
}

from neuron import h
import numpy as np

def reset_simulation():
    h.finitialize(h.v_init)
    h.fcurrent()
    h.frecord_init()
    
    
def i_v_protocol(v_value,v_memb=-70,st_dur1=2.,st_dur2=3.):
    ''' returns current at the soma and m (sodium opening variable) in the end of the axon for given voltage holding potential of soma 
    uses currently defined model in neuron simulator
    '''
    st = h.SEClamp(0.5, sec = h.soma)
    st.dur1 = st_dur1 # ms
    st.dur2 = st_dur2
    st.dur3 = 15.

    st.amp1= v_memb
    st.amp2= v_memb 
    st.amp3= v_value

    st.rs = 0.1 # for ideal clamp set to 0.001

    # record params
    vec_i = h.Vector()
    vec_i.record(st._ref_i) #nA;
        
    vec_axonm = h.Vector()  
    vec_axonm.record(h.axon(0.99)._ref_m_na)

    h.run()    
    
    vec_i = np.array(vec_i)
    vec_axonm = np.array(vec_axonm)
    
    reset_simulation()
    return vec_i, vec_axonm
    
def pn_protocol(v_memb=-70):
    '''use p/n protocol (as defined in 
        * Milescu, Lorin S., Bruce P. Bean, and Jeffrey C. Smith. "Isolation of somatic Na+ currents by selective inactivation of axonal channels with a voltage prepulse." Journal of Neuroscience 30.22 (2010): 7740-7748.
        * Bezanilla, Francisco, and CLAY M. Armstrong. "Inactivation of the sodium channel. I. Sodium current experiments." The Journal of General Physiology 70.5 (1977): 549.
    
    uses currently defined model in neuron simulator

    Returns:
      
      i_soma_pn : somatic current
    '''

    # create stimulation
    st = h.SEClamp(0.5, sec = h.soma)
    st.dur1 = 2. # ms
    st.dur2 = 3.
    st.dur3 = 15.

    st.amp1= v_memb
    st.amp2= v_memb 
    st.amp3= v_memb + 5. # v membrane + pn stimulation

    h.tstop = 20 # ms
    st.rs = 0.1
    vec_i = h.Vector()
    vec_i.record(st._ref_i) #nA;
    h.run()

    i_soma_pn = np.array(vec_i)
    
    reset_simulation()
    
    return i_soma_pn

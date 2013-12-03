function M = getInertia(q)
    global six_link;
    global m_rov;
    M=zeros(12,12);    
    m_manipulator = six_link.inertia(q');
    M=[m_rov,zeros(6,6); zeros(6,6), m_manipulator];
end
function agent = update_agent(agent, u, dt)

    p = agent.p;
    v = agent.v;
    d = agent.d;

    p_new = p + dt * v;
    v_new = v + dt * (u + d);

    agent.p = p_new;
    agent.v = v_new;
    agent.u = u;
end
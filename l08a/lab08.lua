	-- Parametrización y variables de control
	deltatime=5
	side = 0.8
	iter = 1

	-- Obtener los manejadores
	hRob = sim.getObject("/IRB140")
	hMan = sim.getObject("/IRB140/manipulationSphere")
	hDum = sim.getObject("/Dummy")

	-- Posiciones iniciales
	pRob = sim.getObjectPosition(hRob, -1)
	pNew = {pRob[1]+side/2, pRob[2]+side/2, pRob[3]}
	iRes = sim.setObjectPosition(hDum, -1, pNew)
	pDum = sim.getObjectPosition(hDum, -1)



	-- Bucle principal
	while (true) do
		if iter%4 == 1 then 
			pNew[1] = pDum[1] 
			pNew[2] = pDum[2]
		elseif iter%4 == 2 then
			pNew[1] = pDum[1]-side
			pNew[2] = pDum[2];
		elseif iter%4 == 3 then
			pNew[1] = pDum[1]-side
			pNew[2] = pDum[2]-side;
		elseif iter%4 == 0 then
			pNew[1] = pDum[1]
			pNew[2] = pDum[2]-side;
		end
		iter = iter + 1
		iRes = sim.setObjectPosition(hMan, -1, pNew)
		sim.wait(deltatime)
	end




-- hTip = sim.getObject("/IRB140/tip")
-- hTar = sim.getObject("/IRB140/target")
-- pTip = sim.getObjectPosition(hTip, -1)
-- pTar = sim.getObjectPosition(hTar, -1)
-- pMan = sim.getObjectPosition(hMan, -1)
-- pDum[3] = pRob[3]




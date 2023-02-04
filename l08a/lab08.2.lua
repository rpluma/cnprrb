function sysCall_init()
    corout=coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function sysCall_cleanup()
    -- do some clean-up here
end


function coroutineMain()
	-- Parametrización y variables de control
	deltatime=5
	side = 0.8
	iter = 1

	-- Obtener los manejadores
	hRob = sim.getObject("/IRB140")
	hMan = sim.getObject("/IRB140/manipulationSphere")
	hDum = sim.getObject("/Dummy")
	hTip = sim.getObject("/IRB140/tip")
	hTar = sim.getObject("/IRB140/target")

	-- Posiciones iniciales
	pRob = sim.getObjectPosition(hRob, -1)
	pNew = {pRob[1]+side/2, pRob[2]+side/2, pRob[3]}
	iRes = sim.setObjectPosition(hDum, -1, pNew)
	pDum = sim.getObjectPosition(hDum, -1)
	pTar = sim.getObjectPosition(hTar, -1)


	-- Inicialización de Rucking
	--pTip = sim.getObjectPose(hTip, -1)
	pTar = sim.getObjectPose(hTar, -1)
	maxVel = {0.4, 0.4, 0.4, 0.4}
	maxAcc = {0.1, 0.1, 0.1, 0.1}
	maxJrk = {0.1, 0.1, 0.1, 0.1}
	--tarPos = sim.getObjectPose(hTip, -1)
	tarPos = sim.getObjectPose(hTar, -1)
	local function moveToPose_callback(curPose, curVel, curAcc, hTar)
	 	sim.setObjectPose(hTar,-1,curPose)
	end

	-- Bucle principal
	while (true) do
		curPos = sim.getObjectPose(hTip, -1)
		--if curPos[1]==tarPos[1] and curPos[2]==tarPos[2] and curPos[3]==tarPos[3] then
			print(iter)		
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
			tarPos = {pNew[1], pNew[2], pNew[3], pTar[4], pTar[5], pTar[6], pTar[7]}
			sim.moveToPose(-1,curPos,maxVel,maxAcc,maxJrk,tarPos,moveToPose_callback, hTar)
			-- iRes = sim.setObjectPosition(hMan, -1, pNew)
			-- sim.wait(deltatime)
		--end
	end
end


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
	zVac = 0.07 -- Altura de la ventosa
	zCub = 0.10 -- Altura del cubo
	xDif = 0.8  -- Movimiento en el eje x
	back = false -- Regreso a posición inicial

	-- Obtener los manejadores
	hRob = sim.getObject("/IRB140")
	hMan = sim.getObject("/IRB140/manipulationSphere")
	hDum = sim.getObject("/Dummy")
	hTip = sim.getObject("/IRB140/tip")
	hTar = sim.getObject("/IRB140/target")
	hVac = sim.getObject('/IRB140/BaxterVacuumCup')
	hCb0 = sim.getObject('/Cuboid[0]')
	hCb1 = sim.getObject('/Cuboid[1]')
	hCb2 = sim.getObject('/Cuboid[2]')
	aVac = sim.getObjectAlias(hVac, 4)

	-- Posiciones iniciales
	pRob = sim.getObjectPosition(hRob, -1)
	pNew = {pRob[1]+side/2, pRob[2]+side/2, pRob[3]}
	iRes = sim.setObjectPosition(hDum, -1, pNew)
	pDum = sim.getObjectPosition(hDum, -1)
	pTar = sim.getObjectPosition(hTar, -1)
	pLft = sim.getObjectPosition(hCb0, -1)


	-- Inicialización de Rucking
	pTip = sim.getObjectPose(hTip, -1)
	--pTar = sim.getObjectPose(hTar, -1)
	maxVel = {0.4, 0.4, 0.4, 0.4}
	maxAcc = {0.1, 0.1, 0.1, 0.1}
	maxJrk = {0.1, 0.1, 0.1, 0.1}
	local function moveToPose_callback(curPose, curVel, curAcc, hTar)
	 	sim.setObjectPose(hTar,-1,curPose)
	end

	-- Bucle principal
	while (true) do
		-- curPos = sim.getObjectPose(hTar, -1)
		if back then
			pOrg = {pLft[1]-xDif, pLft[2], pLft[3]}
			pDst = {pLft[1],      pLft[2], pLft[3]}
		else
			pOrg = {pLft[1],      pLft[2], pLft[3]}
			pDst = {pLft[1]-xDif, pLft[2], pLft[3]}
		end

		for iCub = 3, 1, -1 do

			-- Cálculos de movimientos
			zOrg = (iCub * zCub) + zVac
			zDst = ((4-iCub)*zCub) + zVac
			zMax = math.max(zOrg, zDst)+0.1

			print(iCub, zOrg, zDst)

			-- Subir por encima de las verticales de los cubos
			curPos = sim.getObjectPose(hTar, -1)
			tarPos = {curPos[1], curPos[2], zMax, curPos[4], curPos[5], curPos[6], curPos[7]}
			sim.moveToPose(-1,curPos,maxVel,maxAcc,maxJrk,tarPos,moveToPose_callback, hTar)
			
			-- Ir hacia el cubo de la izquierda 
			curPos = sim.getObjectPose(hTar, -1)
			tarPos = {pOrg[1], pOrg[2], curPos[3], curPos[4], curPos[5], curPos[6], curPos[7]}
			sim.moveToPose(-1,curPos,maxVel,maxAcc,maxJrk,tarPos,moveToPose_callback, hTar)
						
			-- Bajar el brazo y recogerlo
			curPos = sim.getObjectPose(hTar, -1)
			tarPos = {pOrg[1], pOrg[2], zOrg, curPos[4], curPos[5], curPos[6], curPos[7]}
			sim.moveToPose(-1,curPos,maxVel,maxAcc,maxJrk,tarPos,moveToPose_callback, hTar)
			sim.setInt32Signal(aVac..'_active', 1)
			
			-- Subir el brazo
			curPos = sim.getObjectPose(hTar, -1)
			tarPos[3] = zMax
			sim.moveToPose(-1,curPos,maxVel,maxAcc,maxJrk,tarPos,moveToPose_callback, hTar)
			
			-- Ir hacia la derecha
			curPos = sim.getObjectPose(hTar, -1)
			tarPos = {pDst[1], pDst[2], zMax, curPos[4], curPos[5], curPos[6], curPos[7]}
			sim.moveToPose(-1,curPos,maxVel,maxAcc,maxJrk,tarPos,moveToPose_callback, hTar)
			
			-- Ir hacia abajo y soltarlo
			curPos = sim.getObjectPose(hTar, -1)
			tarPos = {pDst[1], pDst[2], zDst, curPos[4], curPos[5], curPos[6], curPos[7]}
			sim.moveToPose(-1,curPos,maxVel,maxAcc,maxJrk,tarPos,moveToPose_callback, hTar)
			sim.setInt32Signal(aVac..'_active', 0)			
		end
		back = not back

	end
end


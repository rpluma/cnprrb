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
	-- Inicialización
		-- Librería Rucking
			maxVel = {0.4, 0.4, 0.4, 0.4}
			maxAcc = {0.1, 0.1, 0.1, 0.1}
			maxJrk = {0.1, 0.1, 0.1, 0.1}
			local function moveToPoseCb(poseTarget, curVel, curAcc, handTarget)
	 			sim.setObjectPose(handTarget,-1,poseTarget)
			end

		-- Parametrización
			offsetLaser    = 0.02    -- tamaño del láser
			offsetVaccum   = 0.07    -- tamaño de la ventosa
			offsetSecurity = 0.3     -- altura de seguridad para mover el brazo horizontalmente
			offsetCube     = 0.075/2 -- mitad del tamaño del cubo
			thresholdHeigh = 0.3
		-- Manejadores
			handHorizo = sim.getObject("/LaserPointer")
			handVertic = sim.getObject("/IRB140/LaserPointer")
			handTarget = sim.getObject("/IRB140/target")
			handVaccum = sim.getObject('/IRB140/BaxterVacuumCup')
		-- Situación inicicial de la simulación
			aliaVaccum = sim.getObjectAlias(handVaccum, 4)
			posiHoirzo = sim.getObjectPosition(handHorizo, -1)
			stackX     = {0.5, 0.5, 0.5}
			stackY     = {0.0, 0.5, 1.0}
			stackZ     = {0.0, 0.0, 0.0}
			iStackMin  = 1
		-- Resto de variables
			maxHeigh   = 0
		-- Sacar el primer bloque y esperar
			sim.setInt32Signal('producer',1)
			sim.wait(5)
	
	-- Mientras las pilas estén por debajo del umbral
		while (maxHeigh < thresholdHeigh) do
		-- Mover el brazo encima del nuevo bloque
			dist = sim.getFloatSignal('laser_producer_dist',1)
			poseTarget = sim.getObjectPose(handTarget, -1)
			poseNew    = {
				posiHoirzo[1], 
				posiHoirzo[2] + dist + offsetLaser + offsetCube,
				maxHeigh + offsetSecurity,
				poseTarget[4],poseTarget[5],poseTarget[6],poseTarget[7]
				}
			sim.moveToPose(-1,poseTarget,maxVel,maxAcc,maxJrk,poseNew,moveToPoseCb, handTarget)
			
		-- Medir el bloque y visualizar alturas
			dist 	   = sim.getFloatSignal('laser_irb140_dist',1)
			blockHeigh = poseNew[3] - dist - 0.003
			print('Altura='..blockHeigh..'. Pilas = ', stackZ, '. Destino='..iStackMin)
		
		-- Medir el bloque, bajar a por él y cogerlo con la ventosa
			poseTarget = sim.getObjectPose(handTarget, -1)
			poseNew[3] = blockHeigh
			sim.moveToPose(-1,poseTarget,maxVel,maxAcc,maxJrk,poseNew,moveToPoseCb, handTarget)
			sim.setInt32Signal(aliaVaccum..'_active', 1)

		-- Subir el brazo
			poseTarget = sim.getObjectPose(handTarget, -1)
			poseNew[3] = blockHeigh + maxHeigh + offsetSecurity
			sim.moveToPose(-1,poseTarget,maxVel,maxAcc,maxJrk,poseNew,moveToPoseCb, handTarget)

		-- Generar un nuevo bloque
			sim.setInt32Signal('producer',1)

		-- Mover el brazo hacia la pila
			poseTarget = sim.getObjectPose(handTarget, -1)
			poseNew[1] = stackX[iStackMin]
			poseNew[2] = stackY[iStackMin]
			sim.moveToPose(-1,poseTarget,maxVel,maxAcc,maxJrk,poseNew,moveToPoseCb, handTarget)

		-- Bajar el brazo y soltar el bloque
			poseTarget = sim.getObjectPose(handTarget, -1)
			poseNew[3] = stackZ[iStackMin] + blockHeigh
			print(poseNew)
			sim.moveToPose(-1,poseTarget,maxVel,maxAcc,maxJrk,poseNew,moveToPoseCb, handTarget)
			sim.setInt32Signal(aliaVaccum..'_active', 0)

		-- Subir el brazo
			poseTarget = sim.getObjectPose(handTarget, -1)
			poseNew[3] = blockHeigh + maxHeigh + offsetSecurity
			sim.moveToPose(-1,poseTarget,maxVel,maxAcc,maxJrk,poseNew,moveToPoseCb, handTarget)

		-- Actualizar la medición de la pila
			poseTarget = sim.getObjectPose(handTarget, -1)
			sim.wait(1) -- esperar antes de medir
			dist 	   			= sim.getFloatSignal('laser_irb140_dist',1) - 0.003
			print('antes: ', stackZ, poseTarget[3], dist)
			stackZ[iStackMin]   = poseTarget[3] - dist 
			print('despues: ',  stackZ, poseTarget[3], dist)
			

		-- Elegir la pila del siguiente bloque
			maxHeigh = math.max(stackZ[1], stackZ[2], stackZ[3])
			minHeigh = math.min(stackZ[1], stackZ[2], stackZ[3])
			if stackZ[1] == minHeigh then
				iStackMin  = 1
			elseif stackZ[2] == minHeigh then
				iStackMin  = 2
			else
				iStackMin  = 3
			end
		end
	-- Parar la simulación
	sim.stopSimulation()
end


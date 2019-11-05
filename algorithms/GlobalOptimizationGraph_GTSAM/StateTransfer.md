Here We assert SLAM info and AHRS info will never stop.



这里的GPS指的是稳定的GPS信号,误差和延时都有严格要求.

	from	Scene&GPS		!Scene&GPS		!Scene&!GPS		Scene&!GPS
to

Scene&GPS	/			[2]创建Scene的估计GPS(t),	先[1]后[2]	创建Scene的估计GPS(t),
					估计Scene的R(通过AHRS)				平移机身坐标系到GPS.

!Scene&GPS	[3]什么也不做.		/			[1]机身坐标系平移到GPS.	先[3]后[1]

!Scene&!GPS	[3]			[3]			/			[3]

Scene&!GPS	[3]			创建Scene的估计GPS(通	估计Scene的R.机身坐标系	/
					过机身坐标系最后	平移到Scene.
					位置+slam).





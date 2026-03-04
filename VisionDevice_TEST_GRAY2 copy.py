import cv2 
import numpy as np

factor_conversion=0.55/242
valor_umbral=5
cap=cv2.VideoCapture(1)

while True:
    ret,frame=cap.read()
    if not ret:
        break

    gris=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    _,umbral=cv2.threshold(gris,valor_umbral,255,cv2.THRESH_BINARY)
    contornos,_=cv2.findContours(umbral, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if contornos:
        main_contorno=max(contornos,key=cv2.contourArea)
        if cv2.contourArea(main_contorno)>100:
            rect=cv2.minAreaRect(main_contorno)
            cen,tam,ang=rect
            if tam[0]<tam[1]:
                angulo_corregido=ang
            else:
                angulo_corregido=ang+90
            
            M=cv2.getRotationMatrix2D(cen,angulo_corregido,1)
            puntos=main_contorno.reshape(-1,2)
            puntos_homog=np.hstack([puntos,np.ones((puntos.shape[0], 1))])
            puntos_rotados=M.dot(puntos_homog.T).T
            centro_x=cen[0]
            p_izq=puntos_rotados[puntos_rotados[:,0]<centro_x]
            p_der=puntos_rotados[puntos_rotados[:,0]>=centro_x]
            if len(p_izq)>0 and len(p_der)>0:
                p_izq=p_izq[p_izq[:,1].argsort()]
                p_der=p_der[p_der[:,1].argsort()]
                anchos=[]
                for pt_i in p_izq[::10]:
                    dist_y=np.abs(p_der[:,1]-pt_i[1])
                    idx_match=np.argmin(dist_y)
                    if dist_y[idx_match]<2:
                        anchos.append(abs(p_der[idx_match,0]-pt_i[0]))
                if anchos:
                    dist_mm=np.mean(anchos)*factor_conversion
                    cv2.putText(frame, f"Grosor: {dist_mm:.3f} mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow("Grosor", frame)
        cv2.imshow("Umbral", umbral)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()                

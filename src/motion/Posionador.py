import numpy as np

class Posicionador:
    def __init__(self, control, theta_spot_updated, x_spot, y_spot, z_spot, CG, x_offset, b_height):
        self.control = control
        self.theta_spot_updated = theta_spot_updated
        self.x_spot = x_spot
        self.y_spot = y_spot
        self.z_spot = z_spot
        self.CG = CG
        self.x_offset = x_offset
        self.b_height = b_height

    def actualizar_posicion(self):
        # Actualizar ángulo theta
        self.theta_spot_updated[2] += self.control.delta_theta()

        # Matrices de rotación
        Ms_updated = self._calcular_matriz_rotacion_absoluta()
        Msi_updated = self._calcular_matriz_rotacion_local()

        # Calcular compensaciones y posiciones absolutas
        foot_center = self.posicion_abs_pie()
        v_amp_t, kcomp = self.control.compensacion_theta()
        alpha, alphav = self.control.control_fase()
        x_abs_comp, y_abs_comp = self.control.compensacion()

        # Compensaciones en marcos de referencia
        comp = self._calcular_compensacion_local(x_abs_comp, y_abs_comp)
        compt = self._calcular_compensacion_absoluta(comp, kcomp)

        # Posición del centro del marco con compensación
        x_framecenter_comp, y_framecenter_comp, z_framecenter_comp = self._calcular_centro_marco(foot_center, compt)

        # Calcular esquinas del marco en el espacio absoluto
        x_framecorner, y_framecorner, z_framecorner = self._calcular_esquinas_marco(Ms_updated, x_framecenter_comp, y_framecenter_comp, z_framecenter_comp)

        # Calcular posiciones de las patas
        xleg, yleg, zleg, xabs, yabs, zabs = self._calcular_posiciones_patas(
            Msi_updated, Ms_updated, x_framecorner, y_framecorner, z_framecenter_comp,
            x_abs_comp, y_abs_comp, comp, kcomp, alpha, alphav, v_amp_t
        )

        # Actualizar posiciones finales
        x_spot_updated, y_spot_updated, z_spot_updated = self._actualizar_posiciones_finales(
            foot_center, x_framecenter_comp, y_framecenter_comp, z_framecenter_comp,
            xabs, yabs, zabs
        )

        return [xleg, yleg, zleg, self.theta_spot_updated, x_spot_updated, y_spot_updated, z_spot_updated]

    def _calcular_matriz_rotacion_absoluta(self):
        return xyz_rotation_matrix(
            self.theta_spot_updated[3],
            self.theta_spot_updated[4],
            self.theta_spot_updated[2] + self.theta_spot_updated[5],
            False
        )

    def _calcular_matriz_rotacion_local(self):
        return xyz_rotation_matrix(
            -self.theta_spot_updated[3],
            -self.theta_spot_updated[4],
            -(self.theta_spot_updated[2] + self.theta_spot_updated[5]),
            True
        )

    def _calcular_compensacion_local(self, x_abs_comp, y_abs_comp):
        Msi_comp = xyz_rotation_matrix(0, 0, -self.theta_spot_updated[2], True)
        return new_coordinates(
            Msi_comp, x_abs_comp - self.x_spot[0], y_abs_comp - self.y_spot[0], 0, 0, 0, 0
        )

    def _calcular_compensacion_absoluta(self, comp, kcomp):
        Ms_comp = xyz_rotation_matrix(0, 0, self.theta_spot_updated[2], False)
        return new_coordinates(
            Ms_comp, (comp[0] - self.CG[0]) * kcomp + self.x_offset, (comp[1] - self.CG[1]) * kcomp, 0, 0, 0, 0
        )

    def _calcular_centro_marco(self, foot_center, compt):
        x_framecenter_comp = foot_center[0] + compt[0]
        y_framecenter_comp = foot_center[1] + compt[1]
        z_framecenter_comp = self.b_height
        return x_framecenter_comp, y_framecenter_comp, z_framecenter_comp

    def _calcular_esquinas_marco(self, Ms_updated, x_framecenter_comp, y_framecenter_comp, z_framecenter_comp):
        x_frame = [xlf, xrf, xrr, xlr]
        y_frame = [ylf, yrf, yrr, ylr]
        z_frame = [0, 0, 0, 0]

        x_framecorner = np.zeros(4)
        y_framecorner = np.zeros(4)
        z_framecorner = np.zeros(4)

        for i in range(4):
            frame_corner = new_coordinates(
                Ms_updated, x_frame[i], y_frame[i], z_frame[i],
                x_framecenter_comp, y_framecenter_comp, z_framecenter_comp
            )
            x_framecorner[i], y_framecorner[i], z_framecorner[i] = frame_corner

        return x_framecorner, y_framecorner, z_framecorner

    def _calcular_posiciones_patas(self, Msi_updated, Ms_updated, x_framecorner, y_framecorner, z_framecenter_comp,
                                   x_abs_comp, y_abs_comp, comp, kcomp, alpha, alphav, v_amp_t):
        xleg = np.zeros(4)
        yleg = np.zeros(4)
        zleg = np.zeros(4)
        xabs = np.zeros(4)
        yabs = np.zeros(4)
        zabs = np.zeros(4)

        for i in range(4):
            stance = self.control.control_estado()[i]
            if not stance:
                xleg[i], yleg[i], zleg[i], xabs[i], yabs[i], zabs[i] = self._calcular_pata_movimiento(
                    i, Msi_updated, Ms_updated, x_framecorner, y_framecorner, z_framecenter_comp,
                    x_abs_comp, y_abs_comp, comp, kcomp, alpha, alphav, v_amp_t
                )
            else:
                xabs[i], yabs[i], zabs[i] = self.x_spot[i + 2], self.y_spot[i + 2], 0
                xleg[i], yleg[i], zleg[i] = self._calcular_pata_estatica(
                    i, Msi_updated, x_framecorner, y_framecorner, z_framecenter_comp, xabs, yabs, zabs
                )

        return xleg, yleg, zleg, xabs, yabs, zabs

    def _calcular_pata_movimiento(self, i, Msi_updated, Ms_updated, x_framecorner, y_framecorner, z_framecenter_comp,
                                  x_abs_comp, y_abs_comp, comp, kcomp, alpha, alphav, v_amp_t):
        mangle = self.control.velocidad_angular()
        xc, yc = self.control.area_mov.coordenas_circulo()
        radii, an = self.control.area_mov.radio_angulo()

        alphah = an[i] + mangle * alpha[i] * self.cw
        xleg_target = xc + radii[i] * np.cos(alphah) - (comp[0] - self.CG[0]) * kcomp - self.x_offset - x_frame[i]
        yleg_target = yc + radii[i] * np.sin(alphah) - (comp[1] - self.CG[1]) * kcomp - y_frame[i]

        leg_current = new_coordinates(
            Msi_updated, self.x_spot[i + 2] - x_framecorner[i], self.y_spot[i + 2] - y_framecorner[i], -z_framecorner[i], 0, 0, 0
        )

        xint, yint, zint = self._interpolar_posicion(leg_current, xleg_target, yleg_target, v_amp_t, alphav[i])
        legs = new_coordinates(Msi_updated, xint, yint, zint, 0, 0, 0)

        foot_abs = new_coordinates(Ms_updated, legs[0], legs[1], legs[2], x_framecorner[i], y_framecorner[i], z_framecenter_comp)
        return legs[0], legs[1], legs[2], foot_abs[0], foot_abs[1], foot_abs[2]

    def _interpolar_posicion(self, leg_current, xleg_target, yleg_target, v_amp_t, alphav_i):
        seq = ...  # Define seq and stepl based on your logic
        stepl = ...
        tstep = ...

        if (seq[i] + stepl - self.t1) > self.tstep:
            xint = leg_current[0] + (xleg_target - leg_current[0]) * (self.tstep) / (seq[i] + stepl - self.t1)
            yint = leg_current[1] + (yleg_target - leg_current[1]) * (self.tstep) / (seq[i] + stepl - self.t1)
        else:
            xint, yint = xleg_target, yleg_target

        zint = leg_current[2] + v_amp_t * (1 + np.sin(alphav_i)) / 2
        return xint, yint, zint

    def _calcular_pata_estatica(self, i, Msi_updated, x_framecorner, y_framecorner, z_framecenter_comp, xabs, yabs, zabs):
        leg = new_coordinates(
            Msi_updated, xabs[i] - x_framecorner[i], yabs[i] - y_framecorner[i], zabs[i] - z_framecenter_comp, 0, 0, 0
        )
        return leg[0], leg[1], leg[2]

    def _actualizar_posiciones_finales(self, foot_center, x_framecenter_comp, y_framecenter_comp, z_framecenter_comp,
                                       xabs, yabs, zabs):
        x_spot_updated = [
            foot_center[0], x_framecenter_comp, xabs[0], xabs[1], xabs[2], xabs[3],
            self.x_spot[6], self.x_spot[7], self.x_spot[8]
        ]
        y_spot_updated = [
            foot_center[1], y_framecenter_comp, yabs[0], yabs[1], yabs[2], yabs[3],
            self.y_spot[6], self.y_spot[7], self.y_spot[8]
        ]
        z_spot_updated = [
            foot_center[2], z_framecenter_comp, zabs[0], zabs[1], zabs[2], zabs[3],
            self.z_spot[6], self.z_spot[7], self.z_spot[8]
        ]
        return x_spot_updated, y_spot_updated, z_spot_updated
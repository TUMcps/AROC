%% simulateSys
% simulation of the real system behaviour
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [x,t,u] = simulateSys(sys,k,uc,xc,p,tFinal,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function simulates the behaviour of the real system. The value
% for the disturbances are chosen at random
%
%% Input Arguments
%
%
% <html>
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            sys
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            object containing the dynamics of the closed-loop
%            system (class: nonlinParamSys)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            k
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            feedback matrix for the tracking controller
%            (dimension: [nu,nx])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            uc
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference control input (dimension: [nu,1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            xc
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference point of center trajectory
%            (dimension: [nx,1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            p
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial point for the reachable set computation
%            (dimension: [nx,1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            tFinal
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            time horizon for the simulation
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Opts
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a structure containing following options
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .nw
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of system disturbances
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .W
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            set of uncertain disturbances (class: interval)
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% Output Arguments
%
%
% <html>
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            x
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            simulated trajectory
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            t
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            time points for the simulated trajectory
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            u
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            applied control inputs
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <reachsetMPChelp.html |reachsetMPC|>
%

%%
% <html>
%   <hr>
%   <p class="copy">&copy; 2018-2020 I6 Technische Universit&auml;t M&uuml;nchen
%        <tt class="minicdot">&#149;</tt>
%        <a href="https://tumcps.github.io/AROC/">Website</a>
%        <tt class="minicdot">&#149;</tt>
%        <a href="file:txts/LICENSE.txt">License</a>
%   </p>
% <div>
% <table>
%  <tr>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%             <img src="img/logoAroc.png" alt="logoAroc" height="40px">
%      </td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoCora.png" alt="logoCora" height="40px"></td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoChair.png" alt="logoChair" height="40px"></td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoTum.png" alt="logoTum" height="40px"></td>
%  </tr>
% </table>
% </div>
% </html>

%% isLinearModel
% check if a model is linear or nonlinear
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [res,A,B,D,c] = isLinearModel(model,nx,nu,nw) <br>
%        </div>
% </html>
%
%% Description
% Checks if a model is linear of nonlinear. If the model is linear,
% the function returns the system matrices of the linear system
% equation
%
% $\dot x = A \cdot x + B \cdot u + D \cdot w + c$
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
%            model
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to the dynamic function of the model
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            nx
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of system states
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            nu
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of inputs
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            nw
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of disturbances
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
%            res
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            1 if the model is linear, 0 if not
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            A
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            system matrix of the linear model (dimension: [nx,nx])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            B
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input matrix of the linear model (dimension: [nx,nu])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            D
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            disturbance matrix of the linear model (dimension: [nu,nw])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            c
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            constand offset of the linear model (dimension: [nx,1])
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <platoonhelp.html |platoon|>, <carhelp.html |car|>
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

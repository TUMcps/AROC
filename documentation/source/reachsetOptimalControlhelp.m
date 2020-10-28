%% reachsetOptimalControl
% Implementation of the Reachset Optimal Control
% algorithm
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [objContr,res] = reachsetOptimalControl(benchmark,Param,Opts) <br>
%        </div>
% </html>
%
%% Description
% Offline-phase computations for the Reachset Optimal Control
% algorithm.
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
%            benchmark
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            name of the considered benchmark model (see;
%            "aroc/benchmarks/dynamics/...")
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Param
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a structure containing the benchmark parameters
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .R0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial set of states (class: interval)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .xf
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            goal state
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .tFinal
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            final time after which the goal state should be
%            reached
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .U
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            set of admissible control inputs (class:;
%            interval)
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
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Opts
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a structure containing the algorithm settings
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
%            objContr
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            resulting controller storing the data computed during
%            the offline phase (class: objReachsetOptContr)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            res
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            results object storing the computed reachable set and
%            the center trajectory
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <objReachsetOptContrhelp.html |objReachsetOptContr|>
%
%% References
% * *[1] Schuermann et al. (2020)*, Optimal Control Using
% Reachability Analysis
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

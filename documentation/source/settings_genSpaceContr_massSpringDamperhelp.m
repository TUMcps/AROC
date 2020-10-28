%% settings_genSpaceContr_massSpringDamper
% algorithm settings for the
% mass-spring-damper benchmark
%
%% Syntax
%
% <html>
%        <div class="syntax">
% Opts = settings_genSpaceContr_massSpringDamper() <br>
%        </div>
% </html>
%
%% Description
% Algorithm settings and parameter for the controller synthesis
% algorithm that is based on optimal control in generator space for
% the mass-spring-damper benchmark.
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
%            Opts
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a structure containing the algorithm settings
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .N
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of time steps
%            [&#123;10&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .Ninter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of intermediate time steps during one
%            time step
%            [&#123;4&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .reachSteps
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of reachability steps in one time step
%            [&#123;10&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .Q
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            state weighting matrix for the cost function of
%            the optimal control problem
%            [&#123;eye(nx)&#125; / positive-definite square matrix]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input weighting matrix for the cost function of
%            the optimal control problem
%            [&#123;zeros(nu)&#125; / positive-definite square matrix]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .refInput
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            use the input from the reference trajectory
%            as input for the center instead of optimizing
%            [&#123;false&#125; / boolean]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .extHorizon.active
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            use extended optimization horizon for
%            optimal control problems
%            [&#123;false&#125; / true]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .extHorizon.horizon
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            length of the extended optimization
%            horizon in center trajectory time steps
%            [&#123;<tt>'all'</tt>&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .extHorizon.decay
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            decay function for the objective
%            function of the optimization problem
%            with extended optimization horizon
%            [&#123;<tt>'fall+End'</tt>&#125; / <tt>'uniform'</tt> / <tt>'fall'</tt> /;
%            <tt>'fallLinear'</tt> / <tt>'fallLinear+End'</tt> /;
%            <tt>'fallEqDiff'</tt> / <tt>'fallEqDiff+End'</tt> /;
%            <tt>'rise'</tt> / <tt>'quad'</tt> /  <tt>'riseLinear'</tt> /;
%            <tt>'riseEqDiff'</tt> / <tt>'end'</tt>]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .refTraj.Q
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            state weighting matrix for the cost function of
%            optimal control problem (dimension:[nx,nx])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .refTraj.R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input weighting matrix for the cost function of
%            optimal control problem (dimension:[nu,nu])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .refTraj.x
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            user provided reference trajectory
%            (dimension: [nx,Opts.N*Opts.Ninter + 1])
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
%            -.refTraj.u     inputs for the user provided reference
%            trajectory (dimension: [nu,Opts.N*Opts.Ninter])
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.alg
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reachability algorithm that is used
%            [&#123;<tt>'lin'</tt>&#125; / <tt>'poly'</tt>]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.tensorOrder
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            taylor order for the abstraction of
%            the nonlinear function [&#123;2&#125;/ 3]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.taylorTerms
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            taylor order for computing e^At
%            [&#123;20&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.zonotopeOrder
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            upper bound for the zonotope order
%            [&#123;30&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.errorOrder
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            upper bound for the zonotope order
%            before comp. the abstraction error
%            [&#123;5&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.intermediateOrder
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            upper bound for the zonotope order
%            during internal computations
%            [&#123;20&#125; / positive integer]
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
%% See Also
% <generatorSpaceControlhelp.html |generatorSpaceControl|>, <carhelp.html |car|>
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

%% settings_convInterContr_platoon
% algorithm settings for the platoon
% benchmark
%
%% Syntax
%
% <html>
%        <div class="syntax">
% Opts = settings_convInterContr_platoon() <br>
%        </div>
% </html>
%
%% Description
% Algorithm settings and parameter for the Convex Interpolation
% Control Algorithm for the platoon benchmark.
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
%            a structure containing following options
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .controller
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            use exact convex interpolation control law or a
%            linear or quadratic approximation
%            [&#123;<tt>'linear'</tt>&#125; / <tt>'quadratic'</tt> / <tt>'exact'</tt>]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .N
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of time-steps
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
%            [&#123;20&#125; / positive integer]
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
%            .parallel
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            use parallel computing
%            [&#123;false&#125; / true]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .approx.method
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            method used to determine the
%            approximated control law
%            [&#123;<tt>'scaled'</tt>&#125; / <tt>'optimized'</tt> / <tt>'center'</tt>]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .approx.lambda
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            tradeoff betwen vertex inputs and
%            difference from the exact control law
%            [&#123;0.5&#125; / value between 0 and 1]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .polyZono.N
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of reference trajectory time
%            steps after which the polynomial
%            zontope is over-approximated with a
%            parallelotope
%            [&#123;Opts.N/2&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .polyZono.orderDep
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            maximum zonotope order for the
%            dependent part of the polynomial
%            zonotope (for function restructure)
%            [&#123;10&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .polyZono.order
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            maximum zonotope order for the
%            overall polynomial zonotope (for;
%            function restructure)
%            [&#123;20&#125; / positive integer]
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
%            horizon in reference trajectory time
%            steps
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
%            (dimension: [nx,Opts.N + 1])
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
%            -.refTraj.u     inputs for the user provided reference
%            trajectory (dimension: [nu,Opts.N])
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
%            (<tt>'lin'</tt> or <tt>'poly'</tt>)
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
%            the nonlinear function (2 or 3)
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
%            [&#123;100&#125; / positive integer]
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
%            [&#123;50&#125; / positive integer]%
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
% <convexInterpolationControlhelp.html |convexInterpolationControl|>, <carhelp.html |car|>
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

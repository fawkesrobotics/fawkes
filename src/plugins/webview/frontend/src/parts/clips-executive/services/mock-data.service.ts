
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { InMemoryDbService } from 'angular-in-memory-web-api';

export class ClipsExecutiveInMemoryDataService implements InMemoryDbService {
  createDb() {
    const goals =
      [{id: "GOAL-A",
        _class: "GOAL",
        type: "ACHIEVE",
      mode: "FORMULATED"},
       {id: "GOAL-B",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "SELECTED"},
       {id: "GOAL-C",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "EXPANDED",
        plans: ["PLAN-C1"]},
       {id: "GOAL-D",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "COMMITTED"},
       {id: "GOAL-D2",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "FORMULATED",
        parent: "GOAL-D"},
       {id: "GOAL-D2-2",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "FORMULATED",
        parent: "GOAL-D2"},
       {id: "GOAL-D3",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "FORMULATED",
        parent: "GOAL-D"},
       {id: "GOAL-E",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "DISPATCHED"},
       {id: "GOAL-F1",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "FINISHED",
        outcome: "COMPLETED"},
       {id: "GOAL-F2",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "FINISHED",
        outcome: "FAILED",
        message: "Workpiece dropped"},
       {id: "GOAL-G1",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "EVALUATED",
        outcome: "COMPLETED",
        message: ""},
       {id: "GOAL-G2",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "EVALUATED",
        outcome: "FAILED",
        message: "Path is blocked"},
       {id: "GOAL-H",
        _class: "GOAL",
        type: "ACHIEVE",
        mode: "REJECTED",
        message: "Goal does not meet condition X"},
      ];

    const plans = [
      { id: "PLAN-C1",
        kind: "ClipsExecutivePlan",
        apiVersion: "v1beta1",
        "goal-id": "GOAL-C",
        actions: [
          { kind: "ClipsExecutivePlanAction",
            apiVersion: "v1beta1",
            id: "1",
            "operator-name": "goto",
            "param-values": ["START", "C-BS-I"],
            status: "RUNNING",
            executable: true,
            duration: 4.0,
            "dispatch-time": 0.0,
            preconditions: [
              { kind: "DomainPreconditionCompound",
                apiVersion: "v1beta1",
                name: "C-1-1",
                type: "conjunction",
                grounded: true,
                "is-satisfied": true,
                elements: [
                  { kind: "DomainPreconditionAtom",
                    apiVersion: "v1beta1",
                    name: "C-1-1-1",
                    type: "atom",
                    predicate: "said",
                    "param-names": ["text"],
                    "param-values": ["foo"],
                    "param-constants": ["nil"],
                    grounded: true,
                    "is-satisfied": true
                  },
                  { kind: "DomainPreconditionAtom",
                    apiVersion: "v1beta1",
                    name: "C-1-1-2",
                    type: "atom",
                    predicate: "something",
                    "param-names": ["text", "some"],
                    "param-values": ["foo", "bar"],
                    "param-constants": ["nil", "nil"],
                    grounded: true,
                    "is-satisfied": false
                  },
                ]
              },
            ],
            effects: [
              { kind: "DomainEffect",
                apiVersion: "v1beta1",
                name: "C-E-1",
                type: "POSITIVE",
                predicate: "said",
                "param-names": ["text"],
                "param-values": ["hello"],
                "param-constants": ["nil"]
              },
            ],
          },
          { kind: "ClipsExecutivePlanAction",
            apiVersion: "v1beta1",
            id: "1",
            "operator-name": "say",
            "param-values": ["good bye"],
            status: "WAITING",
            executable: true,
            duration: 4.0,
            "dispatch-time": 4.0,
            preconditions: [
              { kind: "DomainPreconditionCompound",
                apiVersion: "v1beta1",
                name: "C-1-1",
                type: "conjunction",
                grounded: true,
                "is-satisfied": false,
                elements: [
                  { kind: "DomainPreconditionAtom",
                    apiVersion: "v1beta1",
                    name: "C-1-1-1",
                    type: "atom",
                    predicate: "said",
                    "param-names": ["text"],
                    "param-values": ["foo"],
                    "param-constants": ["nil"],
                    grounded: true,
                    "is-satisfied": true
                  },
                  { kind: "DomainPreconditionAtom",
                    apiVersion: "v1beta1",
                    name: "C-1-1-2",
                    type: "atom",
                    predicate: "something",
                    "param-names": ["text", "some"],
                    "param-values": ["foo", "bar"],
                    "param-constants": ["nil", "nil"],
                    grounded: true,
                    "is-satisfied": false
                  },
                  { kind: "DomainPreconditionCompound",
                    apiVersion: "v1beta1",
                    name: "C-1-1-3",
                    type: "disjunction",
                    grounded: true,
                    "is-satisfied": false,
                    elements: [
                      { kind: "DomainPreconditionAtom",
                        apiVersion: "v1beta1",
                        name: "C-1-1-3-1",
                        type: "atom",
                        predicate: "said",
                        "param-names": ["text"],
                        "param-values": ["foo"],
                        "param-constants": ["nil"],
                        grounded: true,
                        "is-satisfied": false
                      },
                      { kind: "DomainPreconditionAtom",
                        apiVersion: "v1beta1",
                        name: "C-1-1-3-1",
                        type: "atom",
                        predicate: "something",
                        "param-names": ["text", "some"],
                        "param-values": ["foo", "bar"],
                        "param-constants": ["nil", "nil"],
                        grounded: true,
                        "is-satisfied": false
                      },
                    ]
                  },

                ]
              },
            ],
            effects: []
          }
        ],
        cost: 8.0,
      },
      // { id: "PLAN-C2",
      //   kind: "ClipsExecutivePlan",
      //   apiVersion: "v1beta1",
      //   "goal-id": "GOAL-C",
      //   actions: this.gen_actions(),
      //   cost: 8.0,
      // },
    ];

    const operators = [
      { kind: "DomainOperator",
        apiVersion: "v1beta1",
        name: "say",
        "wait-sensed": false,
        parameters: [
          { name: "text",
            type: "text" },
        ]
      },
      { kind: "DomainOperator",
        apiVersion: "v1beta1",
        name: "goto",
        "wait-sensed": false,
        parameters: [
          { name: "from",
            type: "location" },
          { name: "to",
            type: "location" },
        ]
      },
    ];

    return {goals, plans, "domain-operators": operators};
  }

  gen_actions()
  {
    let actions = [];
    ["FORMULATED", "PENDING", "WAITING", "RUNNING",
     "EXECUTION-SUCCEEDED", "SENSED-EFFECTS-WAIT",
     "SENSED-EFFECTS-HOLD", "EFFECTS-APPLIED", "FINAL",
     "EXECUTION-FAILED", "FAILED"]
      .forEach((s, i) =>
               actions.push(
                 { kind: "ClipsExecutivePlanAction",
                   apiVersion: "v1beta1",
                   id: ""+i,
                   "operator-name": "say",
                   "param-values": ["hello world"],
                   status: s,
                   executable: true,
                   duration: 4.0,
                   "dispatch-time": 0.0
                 }
               ));
    return actions;
  }
}

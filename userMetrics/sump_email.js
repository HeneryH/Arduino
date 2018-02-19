exports.events = {

sumpEmail : { label:'Sump : Email (below 15cm)', 
              icon:'mail', 
              descr:'Send email if water < 15cm below surface', 
              serverExecute:function(node) { 
                     if (node.metrics['CM'] && node.metrics['CM'].value < 15 && 
                        (Date.now() - new Date(node.metrics['CM'].updated).getTime() < 2000)) { 
                        sendEmail('SUMP PUMP ALERT', 'Water is only 15cm below surface and rising - [' + 
                                  node._id + '] ' + node.label.replace(/\{.+\}/ig, '') + ' @ ' + new Date().toLocaleTimeString()); 
                        };
                     }
            },

sumpSMS : { label:'SumpPump : SMS (below 15cm)', 
            icon:'comment', 
            descr:'Send SMS if water < 15cm below surface', 
            serverExecute:function(node) { 
                     if (node.metrics['CM'] && node.metrics['CM'].value < 15 && 
                         (Date.now() - new Date(node.metrics['CM'].updated).getTime() < 2000)) { 
                        sendSMS('SUMP PUMP ALERT', 'Water is only 15cm below surface and rising - [' + 
                                 node._id + '] ' + node.label.replace(/\{.+\}/ig, '') + ' @ ' + new Date().toLocaleTimeString()); 
                        }; 
                     } 
           },

sumpPollAlert : {
  label:'Sump Alert',
  icon:'Check every 5 minutes to see if node is checking in, Alert if last metric is older than 10 min',
  descr:'Sump Alert',
  nextSchedule:function(nodeAtScheduleTime) { return 5*60*1000 /* 5 min */ ; },
  scheduledExecute:function(nodeAtScheduleTime) {
    db.findOne({ _id : nodeAtScheduleTime._id }, function (err, nodeRightNow) {
      if (nodeRightNow) {
        /*just emit a log the status to client(s)*/
        io.sockets.emit('LOG', '** Scheduled Poll Event - Node Metric Value: ' + 
           nodeRightNow.metrics['CM'].value +
           ' last updated:' +
           ((Date.now() - new Date(nodeRightNow.metrics['CM'].updated).getTime()))/1000 +
           ' seconds ago');
      }
      if (nodeRightNow) {
           if (  (Date.now() - new Date(nodeRightNow.metrics['CM'].updated).getTime()) > 10*60*1000 /* 10 min */ )
           {
               io.sockets.emit('LOG', '******* Check in is NOT OK.  *******');
               sendEmail('SUMP PUMP ALERT', 'Sump Pump Node has not checked in in over 10 mins - [' +
                                  nodeRightNow._id + '] ' + nodeRightNow.label.replace(/\{.+\}/ig, '') + ' Now -  ' + new Date().toLocaleTimeString() + ' Last Update -  ' + new Date(nodeRightNow.metrics['CM'].updated).toLocaleTimeString());
           } else
           {
               io.sockets.emit('LOG', '** Check in is OK.');
           }
      }
   });
}},
};

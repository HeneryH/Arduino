exports.events = {
// sumpLostContactEmail : { label:'Sump : Email (Lost contact)', 
//               icon:'mail', 
//               descr:'Send email if not heard from device in 10 minutes', 
//               serverExecute:function(node) { 
//                      if ( (Date.now() - new Date(node.metrics['CM'].updated).getTime() > 2000)) 
//                      { sendEmail('SUMP PUMP ALERT', 'Have not heard from the device in over 10 minutes - [' + node._id + '] ' + node.label.replace(/\{.+\}/ig, '') + ' @ ' + new Date().toLocaleTimeString()); }; } },

sumpEmail : { label:'Sump : Email (below 15cm)', 
              icon:'mail', 
              descr:'Send email if water < 15cm below surface', 
              serverExecute:function(node) { 
                     if (node.metrics['CM'] && node.metrics['CM'].value < 15 && 
                        (Date.now() - new Date(node.metrics['CM'].updated).getTime() < 2000)) 
                     { sendEmail('SUMP PUMP ALERT', 'Water is only 15cm below surface and rising - [' + node._id + '] ' + node.label.replace(/\{.+\}/ig, '') + ' @ ' + new Date().toLocaleTimeString()); }; } },

sumpSMS : { label:'SumpPump : SMS (below 15cm)', icon:'comment', descr:'Send SMS if water < 15cm below surface', serverExecute:function(node) { if (node.metrics['CM'] && node.metrics['CM'].value < 15 && (Date.now() - new Date(node.metrics['CM'].updated).getTime() < 2000)) { sendSMS('SUMP PUMP ALERT', 'Water is only 15cm below surface and rising - [' + node._id + '] ' + node.label.replace(/\{.+\}/ig, '') + ' @ ' + new Date().toLocaleTimeString()); }; } },
};
